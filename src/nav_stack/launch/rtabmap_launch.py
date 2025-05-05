import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_rgbd': True,        # Use RGB-D input (not stereo)
                'subscribe_stereo': False,     # Disable stereo processing
                'subscribe_laser_scan': False,  # Disable laser scan processing
                'frame_id': 'base_link',       # The base frame for robot
                'odom_frame_id': '',           # Empty if you're not using odometry
                'stereo_camera_model': False,  # Disable stereo model processing
                'visual_odometry': True        # Optional, enable visual odometry
            }],
            remappings=[
                ('rgbd_image', '/rgbd_image')
            ]
        )

        # Node(
        #     package='rtabmap_ros',
        #     executable='rtabmap',
        #     name='rtabmap',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'subscribe_rgbd': True
        #     }],
        #     remappings=[
        #         ('rgbd_image', '/rgbd_sync/rgbd_image'),
        #         ('odom', '/odom')
        #     ]
        # )
    ])