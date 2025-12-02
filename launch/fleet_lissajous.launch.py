from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _setup(context, *args, **kwargs):
    count = int(LaunchConfiguration('count').perform(context))
    prefix = LaunchConfiguration('uav_prefix').perform(context)
    names = [f"{prefix}{i}" for i in range(1, count+1)]

    params = {
        'uav_names': names,                     # explicit list
        'center_x': float(LaunchConfiguration('center_x').perform(context)),
        'center_y': float(LaunchConfiguration('center_y').perform(context)),
        'amp_x': float(LaunchConfiguration('amp_x').perform(context)),
        'amp_y': float(LaunchConfiguration('amp_y').perform(context)),
        'a_choices': [1, 2, 3],
        'b_choices': [2, 3, 4],
        'speed_min': float(LaunchConfiguration('speed_min').perform(context)),
        'speed_max': float(LaunchConfiguration('speed_max').perform(context)),
        'altitude_base': float(LaunchConfiguration('altitude_base').perform(context)),
        'altitude_step': float(LaunchConfiguration('altitude_step').perform(context)),
        'rate_hz': float(LaunchConfiguration('rate_hz').perform(context)),
        'k_path': float(LaunchConfiguration('k_path').perform(context)),
        'xy_max': float(LaunchConfiguration('xy_max').perform(context)),
        'kp_z': float(LaunchConfiguration('kp_z').perform(context)),
        'vz_max': float(LaunchConfiguration('vz_max').perform(context)),
        'frame': LaunchConfiguration('frame').perform(context),
        'seed': int(LaunchConfiguration('seed').perform(context)),
        'use_sim_time': True,
    }

    return [Node(
        package='cf_fleet_takeoff',
        executable='cf_fleet_lissajous',
        name='cf_fleet_lissajous',
        output='screen',
        parameters=[params],
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('uav_prefix', default_value='cf'),

        # 4×4 m scenario (amps ≤ 2.0; use 1.8 for margin)
        DeclareLaunchArgument('center_x', default_value='0.0'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('amp_x', default_value='1.8'),
        DeclareLaunchArgument('amp_y', default_value='1.8'),

        # speed band and altitude stacking
        DeclareLaunchArgument('speed_min', default_value='0.1'),
        DeclareLaunchArgument('speed_max', default_value='0.25'),
        DeclareLaunchArgument('altitude_base', default_value='1.0'),
        DeclareLaunchArgument('altitude_step', default_value='0.1'),

        # control & frame
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('k_path', default_value='0.6'),
        DeclareLaunchArgument('xy_max', default_value='0.6'),
        DeclareLaunchArgument('kp_z', default_value='1.2'),
        DeclareLaunchArgument('vz_max', default_value='0.5'),
        DeclareLaunchArgument('frame', default_value='world'),  # 'world' or 'body'
        DeclareLaunchArgument('seed', default_value='42'),

        OpaqueFunction(function=_setup),
    ])

