import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick',
            output='screen', #output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': 'false',
            }],
            remappings=[
                ('/imu/data_raw', '/imu'),
            ]
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen', #output='screen',
            parameters=[{
                'visual_odometry': True,
                'frame_id': 'depth_camera_link',
            }],
            remappings=[
                ('/rgb/image', '/rgb/image_raw'),
                ('/depth/image', '/depth_to_rgb/image_raw'),
                ('/rgb/camera_info', '/rgb/camera_info')
            ]
        ),

        Node(
            package = 'rtabmap_slam',
            executable= 'rtabmap',
            name = 'rtabmap_slam_runner',
            output = 'screen',
            arguments=['-d'], # future me, -d deletes the database on start. ALIAS for --delete_db_on_start
            parameters=[{
                'subscribe_depth' : True,
                'frame_id': 'depth_camera_link',
            }],
            remappings=[
                ('/rgb/image', '/rgb/image_raw'),
                ('/depth/image', '/depth_to_rgb/image_raw'),
                ('/rgb/camera_info', '/rgb/camera_info'),
                ('/imu', '/imu/data')
            ]
        ),
        
        # This is just for testing
        Node(
            package='nav_stack',
            executable='nav_node',
            name='your_node_name',
            output='screen',
        ),
    ])