from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, Shutdown
    
def generate_launch_description():

    ld = LaunchDescription()
    
    
        
    motor_control_server = Node(
        package='humblebot_nodes',
        executable='motor_control_server'
    )

    forward = Node(
        package="humblebot_nodes",
        executable="motor_control_client",
        parameters=[
            {'linear_x': 1.0},
            {'linear_y': 0.0},
            {'angular_z': 0.0},
        ]
    )

    turn_left = Node(
        package="humblebot_nodes",
        executable="motor_control_client",
        parameters=[
            {'linear_x': 0.0},
            {'linear_y': 0.0},
            {'angular_z': (5.75/4)},
        ]
    )

    turn_right = Node(
        package="humblebot_nodes",
        executable="motor_control_client",
        parameters=[
            {'linear_x': -1.0},
            {'linear_y': 0.0},
            {'angular_z': 0.0},
        ]
    )

    backwards = Node(
        package="humblebot_nodes",
        executable="motor_control_client",
        parameters=[
            {'linear_x': 1.0},
            {'linear_y': 0.0},
            {'angular_z': (-5.75/4)},
        ]
    )

    shutdown_timer = TimerAction(
        period=20.0,  # Delay in seconds
        actions=[Shutdown()]  # This will shut down all nodes launched by this launch file
    )

    ld.add_action(motor_control_server)
    ld.add_action(forward)
    ld.add_action(turn_left)
    ld.add_action(turn_right)
    ld.add_action(backwards)
    ld.add_action(shutdown_timer)
    return ld