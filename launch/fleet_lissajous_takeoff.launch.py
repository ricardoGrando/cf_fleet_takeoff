from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _setup(context, *args, **kwargs):
    count = int(LaunchConfiguration('count').perform(context))
    prefix = LaunchConfiguration('uav_prefix').perform(context)
    names = [f"{prefix}{i}" for i in range(1, count+1)]

    P = lambda k: LaunchConfiguration(k).perform(context)

    params = {
        'uav_names': names,
        'frame': P('frame'),
        'rate_hz': float(P('rate_hz')),

        # geometry / path band
        'center_x': float(P('center_x')),
        'center_y': float(P('center_y')),
        'amp_x': float(P('amp_x')),
        'amp_y': float(P('amp_y')),
        'a_choices': [1,2,3],
        'b_choices': [2,3,4],
        'speed_min': float(P('speed_min')),
        'speed_max': float(P('speed_max')),
        'seed': int(P('seed')),

        # altitudes
        'altitude_base': float(P('altitude_base')),
        'altitude_step': float(P('altitude_step')),

        # staged takeoff
        'takeoff_altitude': float(P('takeoff_altitude')),
        'start_delay_s': float(P('start_delay_s')),
        'takeoff_tolerance': float(P('takeoff_tolerance')),
        'takeoff_stable_time': float(P('takeoff_stable_time')),
        'blend_time': float(P('blend_time')),

        # controllers
        'k_path': float(P('k_path')),
        'xy_max': float(P('xy_max')),
        'kp_z': float(P('kp_z')),
        'vz_max': float(P('vz_max')),

        'use_sim_time': True,
    }

    return [Node(
        package='cf_fleet_takeoff',
        executable='cf_fleet_lissajous_takeoff',
        name='cf_fleet_lissajous_takeoff',
        output='screen',
        parameters=[params],
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('uav_prefix', default_value='cf'),
        DeclareLaunchArgument('frame', default_value='body'),

        # 4×4 area envelope
        DeclareLaunchArgument('center_x', default_value='0.0'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('amp_x', default_value='1.8'),
        DeclareLaunchArgument('amp_y', default_value='1.8'),

        # speed band
        DeclareLaunchArgument('speed_min', default_value='0.1'),
        DeclareLaunchArgument('speed_max', default_value='0.25'),
        DeclareLaunchArgument('seed', default_value='42'),

        # altitude stack
        DeclareLaunchArgument('altitude_base', default_value='1.0'),
        DeclareLaunchArgument('altitude_step', default_value='0.1'),

        # staged takeoff
        DeclareLaunchArgument('takeoff_altitude', default_value='0.5'),
        DeclareLaunchArgument('start_delay_s', default_value='3.0'),
        DeclareLaunchArgument('takeoff_tolerance', default_value='0.05'),
        DeclareLaunchArgument('takeoff_stable_time', default_value='0.7'),
        DeclareLaunchArgument('blend_time', default_value='1.5'),

        # control
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('k_path', default_value='0.6'),
        DeclareLaunchArgument('xy_max', default_value='0.6'),
        DeclareLaunchArgument('kp_z', default_value='2.0'),
        DeclareLaunchArgument('vz_max', default_value='1.0'),
        OpaqueFunction(function=_setup),
    ])

