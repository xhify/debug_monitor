"""ROSbridge map update enable/disable control."""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any, Callable

from app_config import DEFAULT_MAP_UPDATE_PARAM, DEFAULT_ROSBRIDGE_HOST, DEFAULT_ROSBRIDGE_PORT


DEFAULT_TIMEOUT_SECONDS = 8.0
MAP_UPDATE_PARAM = DEFAULT_MAP_UPDATE_PARAM


class MappingUpdateError(RuntimeError):
    """Raised when the remote ROS map update parameter cannot be changed."""


@dataclass(frozen=True, slots=True)
class MappingUpdateConfig:
    host: str = DEFAULT_ROSBRIDGE_HOST
    port: int = DEFAULT_ROSBRIDGE_PORT
    parameter: str = DEFAULT_MAP_UPDATE_PARAM
    timeout: float = DEFAULT_TIMEOUT_SECONDS


class MappingUpdateClient:
    """Toggle FAST-LIO mapping updates through ROSbridge."""

    def __init__(
        self,
        config: MappingUpdateConfig | None = None,
        *,
        ros_factory: Callable[[str, int], Any] | None = None,
        param_factory: Callable[[Any, str], Any] | None = None,
        service_factory: Callable[[Any, str, str], Any] | None = None,
        service_request_factory: Callable[[dict], Any] | None = None,
        runner: Callable[..., Any] | None = None,
    ) -> None:
        self.config = config or MappingUpdateConfig()
        self._ros_factory = ros_factory
        self._param_factory = param_factory
        self._service_factory = service_factory
        self._service_request_factory = service_request_factory
        self._runner = runner

    def set_map_update_enabled(self, enabled: bool) -> dict[str, object]:
        self._load_default_factories()
        ros = self._ros_factory(self.config.host, int(self.config.port))
        param_error: Exception | None = None
        try:
            ros.run()
            try:
                self._set_via_param(ros, enabled)
                method = "rosbridge_param"
            except Exception as exc:
                param_error = exc
                self._set_via_rosapi_service(ros, enabled, param_error)
                method = "rosbridge_service"
            return {
                "enabled": bool(enabled),
                "method": method,
                "command": self._command_description(enabled, method),
            }
        except MappingUpdateError:
            raise
        except Exception as exc:
            raise MappingUpdateError(
                f"ROSbridge connection failed for {self.config.host}:{self.config.port}: {exc}"
            ) from exc
        finally:
            close = getattr(ros, "close", None)
            if callable(close):
                close()

    def _set_via_param(self, ros: Any, enabled: bool) -> None:
        if self._param_factory is None:
            raise MappingUpdateError("roslibpy Param is unavailable")
        param = self._param_factory(ros, self.config.parameter)
        param.set(bool(enabled))

    def _set_via_rosapi_service(self, ros: Any, enabled: bool, param_error: Exception | None) -> None:
        if self._service_factory is None:
            raise self._rosapi_error(param_error)
        service = self._service_factory(ros, "/rosapi/set_param", "rosapi/SetParam")
        payload = {
            "name": self.config.parameter,
            "value": json.dumps(bool(enabled)),
        }
        request = self._service_request_factory(payload) if self._service_request_factory else payload
        try:
            response = service.call(request, timeout=float(self.config.timeout))
        except TypeError:
            response = service.call(request)
        except Exception as exc:
            raise self._rosapi_error(param_error, exc) from exc

        if isinstance(response, dict) and response.get("success") is False:
            detail = response.get("message") or response
            raise self._rosapi_error(param_error, RuntimeError(str(detail)))

    def _rosapi_error(
        self,
        param_error: Exception | None,
        service_error: Exception | None = None,
    ) -> MappingUpdateError:
        detail_parts = []
        if param_error is not None:
            detail_parts.append(f"Param: {param_error}")
        if service_error is not None:
            detail_parts.append(f"rosapi: {service_error}")
        detail = "; ".join(detail_parts) or "Param and rosapi service are unavailable"
        return MappingUpdateError(
            f"Failed to set {self.config.parameter} through ROSbridge ({detail}). "
            "Start rosbridge_server and rosapi on the ROS host."
        )

    def _command_description(self, enabled: bool, method: str) -> str:
        value = "true" if enabled else "false"
        return f"rosbridge://{self.config.host}:{self.config.port} {method} {self.config.parameter}={value}"

    def _load_default_factories(self) -> None:
        if self._ros_factory is not None:
            return
        try:
            import roslibpy
        except ImportError as exc:
            raise MappingUpdateError("Missing dependency roslibpy. Install requirements.txt first.") from exc
        self._ros_factory = roslibpy.Ros
        if self._param_factory is None and hasattr(roslibpy, "Param"):
            self._param_factory = roslibpy.Param
        if self._service_factory is None:
            self._service_factory = roslibpy.Service
        if self._service_request_factory is None and hasattr(roslibpy, "ServiceRequest"):
            self._service_request_factory = roslibpy.ServiceRequest
