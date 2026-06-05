import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mapping_update_client import MappingUpdateClient, MappingUpdateConfig, MappingUpdateError


class FakeRos:
    instances = []

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.ran = False
        self.closed = False
        FakeRos.instances.append(self)

    def run(self) -> None:
        self.ran = True

    def close(self) -> None:
        self.closed = True


class FakeParam:
    values = []
    fail = False

    def __init__(self, ros: FakeRos, name: str) -> None:
        self.ros = ros
        self.name = name

    def set(self, value: bool) -> None:
        if FakeParam.fail:
            raise RuntimeError("param unavailable")
        FakeParam.values.append((self.ros.host, self.ros.port, self.name, value))


class FakeService:
    calls = []

    def __init__(self, ros: FakeRos, name: str, service_type: str) -> None:
        self.ros = ros
        self.name = name
        self.service_type = service_type

    def call(self, request, timeout=None):
        FakeService.calls.append((self.name, self.service_type, dict(request), timeout))
        return {"success": True}


class MappingUpdateClientTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeRos.instances = []
        FakeParam.values = []
        FakeParam.fail = False
        FakeService.calls = []

    def test_freeze_and_resume_set_rosbridge_param_without_subprocess_runner(self) -> None:
        def forbidden_runner(*args, **kwargs):
            raise AssertionError("subprocess runner must not be called")

        client = MappingUpdateClient(
            MappingUpdateConfig(host="robot.local", port=19090, parameter="/mapping/map_update_enable", timeout=3.5),
            ros_factory=FakeRos,
            param_factory=FakeParam,
            runner=forbidden_runner,
        )

        frozen = client.set_map_update_enabled(False)
        resumed = client.set_map_update_enabled(True)

        self.assertFalse(frozen["enabled"])
        self.assertTrue(resumed["enabled"])
        self.assertEqual(frozen["method"], "rosbridge_param")
        self.assertEqual(resumed["method"], "rosbridge_param")
        self.assertEqual(
            FakeParam.values,
            [
                ("robot.local", 19090, "/mapping/map_update_enable", False),
                ("robot.local", 19090, "/mapping/map_update_enable", True),
            ],
        )
        self.assertTrue(all(ros.closed for ros in FakeRos.instances))
        self.assertNotIn("ssh", str(frozen["command"]))

    def test_falls_back_to_rosapi_set_param_service_when_param_fails(self) -> None:
        FakeParam.fail = True
        client = MappingUpdateClient(
            MappingUpdateConfig(host="robot.local", port=9090, parameter="/mapping/map_update_enable", timeout=2.0),
            ros_factory=FakeRos,
            param_factory=FakeParam,
            service_factory=FakeService,
        )

        result = client.set_map_update_enabled(False)

        self.assertEqual(result["method"], "rosbridge_service")
        self.assertEqual(
            FakeService.calls,
            [
                (
                    "/rosapi/set_param",
                    "rosapi/SetParam",
                    {"name": "/mapping/map_update_enable", "value": "false"},
                    2.0,
                )
            ],
        )

    def test_rosapi_failure_explains_required_host_services(self) -> None:
        class FailingService(FakeService):
            def call(self, request, timeout=None):
                raise RuntimeError("service missing")

        FakeParam.fail = True
        client = MappingUpdateClient(
            MappingUpdateConfig(host="robot.local", port=9090),
            ros_factory=FakeRos,
            param_factory=FakeParam,
            service_factory=FailingService,
        )

        with self.assertRaises(MappingUpdateError) as ctx:
            client.set_map_update_enabled(False)

        message = str(ctx.exception)
        self.assertIn("rosapi", message)
        self.assertIn("rosbridge", message)
        self.assertIn("/mapping/map_update_enable", message)


if __name__ == "__main__":
    unittest.main()
