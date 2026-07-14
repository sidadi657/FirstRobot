import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'forros_description'
    pkg_share = get_package_share_directory(pkg_name)

    yaml_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + os.path.dirname(pkg_share)
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(pkg_share)

    xacro_file = os.path.join(pkg_share, 'urdf', 'forros.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'controller_params_file': yaml_file}).toxml()

   
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r '+os.path.join(pkg_share, 'worlds', 'my_world.sdf')}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'forros', '-z', '0.32'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--param-file', yaml_file],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller', '--controller-manager', '/controller_manager', '--param-file', yaml_file],
        output='screen'
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_velocity_controller', '--controller-manager', '/controller_manager', '--param-file', yaml_file],
        output='screen'
    )
    

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            steering_controller_spawner,
            velocity_controller_spawner,
        ]
    )

    config_dir = os.path.join(get_package_share_directory('forros_description'), 'config')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        parameters=[os.path.join(config_dir, 'twist_mux.yaml')]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        bridge,
        delayed_controllers,
        twist_mux_node,
    ])