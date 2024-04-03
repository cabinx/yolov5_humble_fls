import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yolov5_humble_fls'),
        'config',
        'yolov5_humble_fls.yaml'
    )
    
    return LaunchDescription([
      Node(
        package='yolov5_humble_fls',  
        executable='yolov5_ros2', 
        namespace='yolov5_humble_fls',
        name='yolov5_ros2',
        parameters=[config],
        output="screen",
      )
    ])