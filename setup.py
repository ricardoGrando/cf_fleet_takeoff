from setuptools import setup

package_name = 'cf_fleet_takeoff'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/fleet_takeoff.launch.py']),
        ('share/' + package_name + '/launch', ['launch/fleet_circle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ricardo',
    maintainer_email='you@example.com',
    description='Take off and hover at 1 m for a fleet of Crazyflies using cmd_vel and odom feedback.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cf_fleet_takeoff = cf_fleet_takeoff.takeoff_node:main',
            'cf_fleet_circle  = cf_fleet_takeoff.circle_node:main',
        ],
    },
)

