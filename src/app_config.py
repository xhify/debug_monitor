"""Centralized environment-backed defaults for ROSbridge integrations."""

from __future__ import annotations

import os


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None or not value.strip():
        return default
    try:
        return int(value)
    except ValueError:
        return default


DEFAULT_ROSBRIDGE_HOST = os.getenv("DEBUG_MONITOR_ROSBRIDGE_HOST", "192.168.0.14")
DEFAULT_ROSBRIDGE_PORT = _env_int("DEBUG_MONITOR_ROSBRIDGE_PORT", 9090)
DEFAULT_FASTLIO_ODOM_TOPIC = os.getenv("DEBUG_MONITOR_FASTLIO_ODOM_TOPIC", "/Odometry")
DEFAULT_MAP_TOPIC = os.getenv("DEBUG_MONITOR_MAP_TOPIC", "/Laser_map")
DEFAULT_MAP_UPDATE_PARAM = os.getenv("DEBUG_MONITOR_MAP_UPDATE_PARAM", "/mapping/map_update_enable")
DEFAULT_RECORDINGS_DIR = os.getenv("DEBUG_MONITOR_RECORDINGS_DIR", r"D:\debug_monitor\recordings")

ROSBRIDGE_DATA_TOPIC_OPTIONS = (
    {"topic": "/odom", "type": "nav_msgs/Odometry", "label": "里程计 /odom"},
    {"topic": DEFAULT_FASTLIO_ODOM_TOPIC, "type": "nav_msgs/Odometry", "label": "FAST-LIO 里程计"},
    {"topic": "/imu", "type": "sensor_msgs/Imu", "label": "IMU /imu"},
    {"topic": "/active_imu", "type": "sensor_msgs/Imu", "label": "Active IMU /active_imu"},
    {"topic": "/PowerVoltage", "type": "std_msgs/Float32", "label": "电压 /PowerVoltage"},
    {"topic": "/wheeltec/akm_state", "type": "turn_on_wheeltec_robot/AkmState", "label": "Ackermann 状态"},
    {"topic": "/wheeltec/control_debug", "type": "turn_on_wheeltec_robot/ControlDebug", "label": "控制调试"},
    {"topic": "/wheeltec/chassis_diagnostics", "type": "turn_on_wheeltec_robot/ChassisDiagnostics", "label": "底盘诊断"},
)
ROSBRIDGE_DATA_TOPIC_PRESETS = {
    "none": [],
    "basic_low_bandwidth": ["/odom", "/imu"],
    "localization": [DEFAULT_FASTLIO_ODOM_TOPIC, "/imu", "/active_imu"],
    "diagnostics": [
        "/PowerVoltage",
        "/wheeltec/akm_state",
        "/wheeltec/control_debug",
        "/wheeltec/chassis_diagnostics",
    ],
    "full": [option["topic"] for option in ROSBRIDGE_DATA_TOPIC_OPTIONS],
}

DEFAULT_ROSBAG_REMOTE_DIR = "/home/wheeltec/bags"
DEFAULT_ROSBAG_SPLIT_SIZE_MB = 2048
DEFAULT_ROSBAG_COMPRESSION = "lz4"

ROSBAG_TOPIC_PRESETS = {
    "control": [
        "/imu",
        "/Odometry",
        "/cmd_vel",
        "/line_follow_control",
        "/tf",
        "/tf_static",
    ],
    "fastlio": [
        "/point_cloud_filtered",
        "/imu",
        "/Odometry",
        "/path",
        "/cmd_vel",
        "/tf",
        "/tf_static",
    ],
    "trajectory_environment": [
        "/point_cloud_filtered",
        "/Laser_map",
        "/imu",
        "/active_imu",
        "/Odometry",
        "/odom",
        "/path",
        "/tf",
        "/tf_static",
        "/cmd_vel",
        "/wheeltec/akm_state",
        "/PowerVoltage",
    ],
    "fallback_no_fastlio": [
        "/point_cloud_filtered",
        "/imu",
        "/active_imu",
        "/odom",
        "/tf",
        "/tf_static",
        "/cmd_vel",
        "/line_follow_control",
        "/PowerVoltage",
        "/wheeltec/akm_state",
        "/wheeltec/control_debug",
        "/wheeltec/chassis_diagnostics",
    ],
    "full": [
        "/point_cloud_filtered",
        "/imu",
        "/active_imu",
        "/odom",
        "/Odometry",
        "/path",
        "/cmd_vel",
        "/line_follow_control",
        "/PowerVoltage",
        "/wheeltec/akm_state",
        "/wheeltec/control_debug",
        "/wheeltec/chassis_diagnostics",
        "/tf",
        "/tf_static",
    ],
}
