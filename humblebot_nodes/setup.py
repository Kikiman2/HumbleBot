from setuptools import find_packages, setup

package_name = 'humblebot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='en',
    maintainer_email='gati.krisztian10@gmail.com',
    description='Contains all the nodes for the Humble bot to work.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "velocity_control=humblebot_nodes.wheel_velocities:main",
            "direction_publisher=humblebot_nodes.send_robot_directions:main",
            "motor_control_server=humblebot_nodes.stepper_vel_action_server:main",
            "motor_control_client=humblebot_nodes.stepper_vel_action_client:main",
            "stop_rplidar_service=humblebot_nodes.stop_rplidar:main",
            "start_rplidar_service=humblebot_nodes.start_rplidar:main"
        ],
    },
)
