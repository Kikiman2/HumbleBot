from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():

    ld = LaunchDescription()

    start_camera = Node(
    package='v4l2_camera',
    executable='v4l2_camera_node',
    output='screen',
    parameters=[{
        'image_size': [1280,720],
        'camera_frame_id': 'camera_link_optical'
    }]
    )

    start_image_viewer = Node(
    package='rqt_image_view',
    executable='rqt_image_view',
    output='screen',
    )


    ld.add_action(start_camera)
    ld.add_action(start_image_viewer)
    return ld