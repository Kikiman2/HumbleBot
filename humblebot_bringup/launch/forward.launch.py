from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, Shutdown
    
def generate_launch_description():

    ld = LaunchDescription()
    
    
        
    motor_control_server = Node(
        package='humblebot_nodes',
        executable='motor_control_server'
    )

    motor_control_client = Node(
        package="humblebot_nodes",
        executable="motor_control_client",
        parameters=[
            {'linear_x': 1.0},
            {'linear_y': 0.0},
            {'angular_z': 0.0},
        ]
    )

    shutdown_timer = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[Shutdown()]  # This will shut down all nodes launched by this launch file
    )

    ld.add_action(motor_control_server)
    ld.add_action(motor_control_client)
    ld.add_action(shutdown_timer)
    return ld