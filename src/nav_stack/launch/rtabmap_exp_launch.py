import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node, PushRosNamespace # Removed LoadComposableNode
from launch_ros.descriptions import ComposableNode # Correct import for ComposableNode

# Keep the launch_setup function structure as before...

def launch_setup(context, *args, **kwargs):
    # --- Declare Launch Arguments ---
    # (Keep this part the same)
    color_enabled = LaunchConfiguration('color_enabled').perform(context) == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    qos = LaunchConfiguration('qos').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    # --- Get Package Paths ---
    # (Keep this part the same)
    pkg_azure_kinect_ros2_driver = get_package_share_directory('azure_kinect_ros2_driver')
    pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')
    urdf_file = os.path.join(pkg_azure_kinect_ros2_driver, 'urdf', 'azure_kinect.urdf.xacro')


    # --- Robot Description ---
    # (Keep this part the same)
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description_param = {"robot_description": robot_description_content, 'use_sim_time': use_sim_time}

    # --- TF Publishers ---
    # (Keep this part the same)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # --- Define Composable Nodes ---
    # Define the kinect driver component
    kinect_driver_component = ComposableNode(
        package='azure_kinect_ros2_driver',
        plugin='azure_kinect_ros2_driver::AzureKinectROS2DriverNode',
        name='k4a', # Node name within the container
        namespace='', # Keep topics like /depth/image_raw, /rgb/image_raw
        parameters=[{
            'depth_enabled': True,
            'depth_mode': 'WFOV_2X2BINNED',
            'color_enabled': color_enabled,
            'color_resolution': '720P',
            'fps': 30,
            'point_cloud': False,
            'rgb_point_cloud': False,
            'imu_rate_target': 100,
            'rescale_ir_to_mono8': True,
            'tf_prefix': '',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            # ('/imu', '/k4a/imu') # Optional remapping if needed
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Define the RGB rectification component (conditionally included later)
    rectify_rgb_component = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_rgb_node',
        namespace='rgb',
        remappings=[
            ('image', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Define the IR rectification component (conditionally included later)
    rectify_ir_component = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_ir_node',
        namespace='ir',
        remappings=[
            ('image', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # --- Create the list of components to load based on condition ---
    composable_node_descriptions=[kinect_driver_component]
    if color_enabled:
        composable_node_descriptions.append(rectify_rgb_component)
    else:
        composable_node_descriptions.append(rectify_ir_component)


    # --- Launch Component Container ---
    # This container Node will load the components defined above
    kinect_container = Node(
        package='rclcpp_components',
        # Use 'component_container_mt' for multi-threaded execution
        # Use 'component_container' for single-threaded execution
        executable='component_container_mt',
        name='kinect_container',
        namespace='', # Global container
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=composable_node_descriptions, # Pass the list here
    )


    # --- IMU Filter ---
    # (Keep this part the same)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'camera_base',
            'qos_imu': qos,
        }],
        remappings=[('imu/data_raw', '/imu')] # Remap driver's IMU topic
    )

    # --- RTAB-Map SLAM Node ---
    # (Keep the parameter definitions and remappings the same)
    rtabmap_common_params = {
        'frame_id': 'camera_base',
        'subscribe_depth': True,
        'subscribe_rgb': color_enabled,
        'subscribe_stereo': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'subscribe_user_data': False,
        'subscribe_odom_info': False,
        'subscribe_imu': True,
        'approx_sync': True,
        'use_sim_time': use_sim_time,
        'qos_image': qos,
        'qos_imu': qos,
        'qos_camera_info': qos,
        'queue_size': 30,
        'Vis/MaxFeatures': 1000,
        'Vis/MinInliers': 12,
        'delete_db_on_start': True,
        'Optimizer/GravitySigma': 0.3,
        'wait_imu_to_init': True,
    }

    if color_enabled:
        rtabmap_conditional_params = {
            'GFTT/MinDistance': 7.0,
            'Vis/CorGuessWinSize': 40,
        }
        rtabmap_remappings = [
            ('rgb/image', '/rgb/image_rect'),
            ('depth/image', '/depth_to_color/image_raw'), # Check this topic name from your driver
            ('rgb/camera_info', '/rgb/camera_info'),
            ('imu', '/imu/data')
        ]
    else: # IR mode
        rtabmap_conditional_params = {}
        rtabmap_remappings = [
            ('rgb/image', '/ir/image_rect'),
            ('depth/image', '/depth/image_raw'), # Assuming raw depth for IR mode
            ('rgb/camera_info', '/ir/camera_info'),
            ('imu', '/imu/data')
        ]

    rtabmap_params = {**rtabmap_common_params, **rtabmap_conditional_params}

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        arguments=['-d', '--ros-args', '--log-level', log_level]
    )

    # --- List of actions to return ---
    # Now kinect_container takes care of loading the components
    actions = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        kinect_container, # This Node launches the container and loads components
        imu_filter_node,
        rtabmap_slam_node,
    ]

    return actions


def generate_launch_description():
    # Keep the argument declarations the same
    return LaunchDescription([
        DeclareLaunchArgument(
            'color_enabled', default_value='true',
            description='Use color camera stream (true) or IR stream (false)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS profile for sensor data (depth=1, other=2(default)) Note: Use string tokens ("reliable", "best_effort") or integer representations.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Logging level (debug, info, warn, error, fatal)'),

        OpaqueFunction(function=launch_setup)
    ])