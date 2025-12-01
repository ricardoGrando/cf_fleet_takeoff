from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    actions = []
    count = int(LaunchConfiguration('count').perform(context))
    prefix = LaunchConfiguration('uav_prefix').perform(context)

    params = {
        'uav_names': [f"{prefix}{i}" for i in range(1, count + 1)],
        'target_altitude': float(LaunchConfiguration('target_altitude').perform(context)),
        'kp': float(LaunchConfiguration('kp').perform(context)),
        'vz_max': float(LaunchConfiguration('vz_max').perform(context)),
        'stable_tolerance': float(LaunchConfiguration('stable_tolerance').perform(context)),
        'stable_time': float(LaunchConfiguration('stable_time').perform(context)),
        'rate_hz': float(LaunchConfiguration('rate_hz').perform(context)),
        'use_sim_time': True,
    }

    actions.append(Node(
        package='cf_fleet_takeoff',
        executable='cf_fleet_takeoff',
        name='cf_fleet_takeoff',
        output='screen',
        parameters=[params],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('uav_prefix', default_value='cf'),
        DeclareLaunchArgument('target_altitude', default_value='1.0'),
        DeclareLaunchArgument('kp', default_value='1.2'),
        DeclareLaunchArgument('vz_max', default_value='0.7'),
        DeclareLaunchArgument('stable_tolerance', default_value='0.05'),
        DeclareLaunchArgument('stable_time', default_value='1.0'),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        OpaqueFunction(function=_setup),
    ])

