from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, Shutdown
    
def generate_launch_description():

    ld = LaunchDescription()
    
    
        
    micro_ros_communication = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0']
    )

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
            {'linear_x': 1.0},
            {'linear_y': 0.0},
            {'angular_z': 0.0},
        ]
    )

    shutdown_timer = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[Shutdown()]  # This will shut down all nodes launched by this launch file
    )

    ld.add_action(micro_ros_communication)
    ld.add_action(wheel_velocities)
    ld.add_action(send_robot_directions)
    ld.add_action(shutdown_timer)
    return ld