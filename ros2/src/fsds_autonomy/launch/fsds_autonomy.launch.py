from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("fsds_autonomy"))
    config_path = share_dir / "config" / "autonomy.yaml"
    rc_model = Path("/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt")
    model_default = str(rc_model) if rc_model.exists() else ""
    map_default = os.path.join(os.path.expanduser("~"), ".fsds_autonomy", "maps")
    dataset_default = os.path.join(os.path.expanduser("~"), ".fsds_autonomy", "datasets", "fsds_cones")

    config = LaunchConfiguration("config")
    map_dir = LaunchConfiguration("map_dir")
    model_path = LaunchConfiguration("model_path")
    dataset_dir = LaunchConfiguration("dataset_dir")
    dataset_enabled = LaunchConfiguration("dataset_enabled")
    use_testing_odom = LaunchConfiguration("use_testing_odom")
    auto_reset_enabled = LaunchConfiguration("auto_reset_enabled")
    control_enabled = LaunchConfiguration("control_enabled")

    common_params = [config]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(config_path)),
            DeclareLaunchArgument("map_dir", default_value=map_default),
            DeclareLaunchArgument("model_path", default_value=model_default),
            DeclareLaunchArgument("dataset_dir", default_value=dataset_default),
            DeclareLaunchArgument("dataset_enabled", default_value="false"),
            DeclareLaunchArgument("use_testing_odom", default_value="false"),
            DeclareLaunchArgument("auto_reset_enabled", default_value="true"),
            DeclareLaunchArgument(
                "control_enabled",
                default_value="true",
                description="Set false for manual driving with autonomy perception, mapping, and camera preview only.",
            ),
            Node(
                package="fsds_autonomy",
                executable="mission_manager",
                name="fsds_mission_manager",
                output="screen",
                parameters=[config, {"map_dir": map_dir}],
            ),
            Node(
                package="fsds_autonomy",
                executable="state_estimator",
                name="fsds_state_estimator",
                output="screen",
                parameters=[config, {"use_testing_odom": use_testing_odom}],
            ),
            Node(
                package="fsds_autonomy",
                executable="lidar_cone_detector",
                name="fsds_lidar_cone_detector",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="fsds_autonomy",
                executable="camera_detector",
                name="fsds_camera_detector",
                output="screen",
                parameters=[config, {"model_path": model_path}],
            ),
            Node(
                package="fsds_autonomy",
                executable="sensor_fusion",
                name="fsds_sensor_fusion",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="fsds_autonomy",
                executable="mapper",
                name="fsds_mapper",
                output="screen",
                parameters=[config, {"map_dir": map_dir}],
            ),
            Node(
                package="fsds_autonomy",
                executable="raceline_planner",
                name="fsds_raceline_planner",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="fsds_autonomy",
                executable="behavior_planner",
                name="fsds_behavior_planner",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="fsds_autonomy",
                executable="controller",
                name="fsds_controller",
                output="screen",
                parameters=common_params,
                condition=IfCondition(control_enabled),
            ),
            Node(
                package="fsds_autonomy",
                executable="dataset_builder",
                name="fsds_dataset_builder_cam1",
                output="screen",
                parameters=[
                    config,
                    {
                        "enabled": dataset_enabled,
                        "dataset_dir": dataset_dir,
                        "camera_topic": "/fsds/cam1/image_color",
                        "camera_name": "cam1",
                    },
                ],
            ),
            Node(
                package="fsds_autonomy",
                executable="dataset_builder",
                name="fsds_dataset_builder_cam2",
                output="screen",
                parameters=[
                    config,
                    {
                        "enabled": dataset_enabled,
                        "dataset_dir": dataset_dir,
                        "camera_topic": "/fsds/cam2/image_color",
                        "camera_name": "cam2",
                    },
                ],
            ),
            Node(
                package="fsds_autonomy",
                executable="foxglove_visualizer",
                name="fsds_foxglove_visualizer",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="fsds_autonomy",
                executable="offtrack_reset_monitor",
                name="fsds_offtrack_reset_monitor",
                output="screen",
                parameters=[config, {"enabled": auto_reset_enabled}],
            ),
        ]
    )
