from launch import LaunchDescription
from launch_ros.actions import Node

    
def generate_launch_description():

    ld = LaunchDescription()

    wheel_velocities = Node(
        package="humblebot_nodes",
        executable="velocity_control"  
    )

    send_robot_directions = Node(
        package="humblebot_nodes",
        executable="direction_publisher",
        output='screen',
        emulate_tty=True,
        parameters=[
            {'linear_x': 0.0},
            {'linear_y': 1.0},
            {'angular_x': 0.0}
        ]
    )

    ld.add_action(wheel_velocities)
    ld.add_action(send_robot_directions)
    return ld