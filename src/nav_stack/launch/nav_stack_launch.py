import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Include the launch file from the rtabmap_launch package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py'),
            launch_arguments={
                'rgb_topic': '/rgb/image_raw', # Remapping input topic
            }.items()
        ),
        
        # You can add your own nodes here, if needed
        Node(
            package='nav_stack',
            executable='nav_node',
            name='your_node_name',
            output='screen',
        ),
    ])