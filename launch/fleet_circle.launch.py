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
        'speed': float(LaunchConfiguration('speed').perform(context)),
        'spacing': float(LaunchConfiguration('spacing').perform(context)),
        'radius': float(LaunchConfiguration('radius').perform(context)),
        'center_x': float(LaunchConfiguration('center_x').perform(context)),
        'center_y': float(LaunchConfiguration('center_y').perform(context)),
        'altitude': float(LaunchConfiguration('altitude').perform(context)),
        'rate_hz': float(LaunchConfiguration('rate_hz').perform(context)),
        'k_error_xy': float(LaunchConfiguration('k_error_xy').perform(context)),
        'xy_max': float(LaunchConfiguration('xy_max').perform(context)),
        'kp_z': float(LaunchConfiguration('kp_z').perform(context)),
        'vz_max': float(LaunchConfiguration('vz_max').perform(context)),
        'frame': LaunchConfiguration('frame').perform(context),
        'use_sim_time': True,
    }

    actions.append(Node(
        package='cf_fleet_takeoff',
        executable='cf_fleet_circle',
        name='cf_fleet_circle',
        output='screen',
        parameters=[params]
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='5'),
        DeclareLaunchArgument('uav_prefix', default_value='cf'),

        DeclareLaunchArgument('speed', default_value='0.1'),
        DeclareLaunchArgument('spacing', default_value='0.2'),
        DeclareLaunchArgument('radius', default_value='2.0'),
        DeclareLaunchArgument('center_x', default_value='0.0'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('altitude', default_value='1.0'),

        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('k_error_xy', default_value='0.5'),
        DeclareLaunchArgument('xy_max', default_value='0.6'),
        DeclareLaunchArgument('kp_z', default_value='1.2'),
        DeclareLaunchArgument('vz_max', default_value='0.7'),
        DeclareLaunchArgument('frame', default_value='world'),  # 'world' or 'body'

        OpaqueFunction(function=_setup),
    ])

