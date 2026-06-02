import json
from os.path import expanduser

import launch
import launch_ros.actions

CAMERA_FRAMERATE = 30.0


def camera_nodes_from_settings(context):
    settings_path = launch.substitutions.LaunchConfiguration('settings_path').perform(context)
    with open(settings_path, 'r') as file:
        settings = json.load(file)

    camera_configs = settings.get('Vehicles', {}).get('FSCar', {}).get('Cameras', {})
    if(not camera_configs):
        print(f'no cameras configured in {settings_path}')

    return [
        launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge_camera',
            namespace="fsds/camera", 
            name=camera_name,
            output='screen',
            parameters=[
                {'camera_name': camera_name},
                {'depthcamera': camera_config["CaptureSettings"][0]["ImageType"] == 2},
                {'framerate': CAMERA_FRAMERATE},
                {'host_ip': launch.substitutions.LaunchConfiguration('host')},
                {'api_port': launch.substitutions.LaunchConfiguration('api_port')},
                {'timeout': launch.substitutions.LaunchConfiguration('timeout')},
            ]
        ) for camera_name, camera_config in camera_configs.items()]


def generate_launch_description():
    default_settings_path = expanduser("~") + '/Formula-Student-Driverless-Simulator/settings.json'

    ld = launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='settings_path',
            default_value=default_settings_path
        ),
        launch.actions.DeclareLaunchArgument(
            name='api_port',
            default_value='41451'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mission_name',
            default_value='trackdrive'
        ),
        launch.actions.DeclareLaunchArgument(
            name='track_name',
            default_value='A'
        ),
        launch.actions.DeclareLaunchArgument(
            name='competition_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='manual_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='UDP_control',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='30.0'
        ),
        launch.actions.OpaqueFunction(function=camera_nodes_from_settings),
        launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge',
            name='ros_bridge',
            namespace='fsds',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'update_odom_every_n_sec': 0.004
                },
                {
                    'update_imu_every_n_sec': 0.004
                },
                {
                    'update_gps_every_n_sec': 0.1
                },
                {
                    'update_gss_every_n_sec': 0.01
                },
                {
                    'publish_static_tf_every_n_sec': 1.0
                },
                {
                    'update_lidar_every_n_sec': 0.1
                },
                {
                    'host_ip': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'api_port': launch.substitutions.LaunchConfiguration('api_port')
                },
                {
                    'mission_name': launch.substitutions.LaunchConfiguration('mission_name')
                },
                {
                    'track_name': launch.substitutions.LaunchConfiguration('track_name')
                },
                {
                    'competition_mode': launch.substitutions.LaunchConfiguration('competition_mode')
                },
                {
                    'manual_mode': launch.substitutions.LaunchConfiguration('manual_mode')
                },
                {
                    'UDP_control': launch.substitutions.LaunchConfiguration('UDP_control')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
