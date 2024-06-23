import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='nav_slam_world')
    model_path = "/home/hiro/simple_world4ignition_gazebo/models/"

    #ignition gazeboがモデルにアクセスできるように設定
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/iron", "share"),
        ":" +
        model_path]
    )

    #ロボットをスポーンさせる設定
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'f1d2', 
            '-name', 'f1d2', 
            '-file', PathJoinSubstitution([model_path, "robot.sdf"]),
            #ロボットの位置を指定
           '-allow_renaming', 'true',
           '-x', '0',
           '-y', '0',
           '-z', '0'],
    )
    
    #フィールドをスポーンさせる設定
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
            #フィールドのsdfファイルを指定
        arguments=['-file', PathJoinSubstitution([model_path,"field.sdf"]),
                   '-allow_renaming', 'false'],
    )

    bridge_ign2ros2 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera_front/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/model/f1d2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/f1d2/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/model/f1d2/odometry', '/odom'),
            ('/model/f1d2/tf', '/tf')
        ],
    )

    base_link2base_footprint_tf = Node(package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "-0.4", "0", "0", "0", "base_link", "base_footprint"]
    )

    lidar_box2gpu_lidar_tf = Node(package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0.0", "0", "0", "0", "lidar_box", "f1d2/lidar_box/gpu_lidar"]
    )
    
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twistkeyboard',
        prefix="xterm -e",
    )

    sdf = os.path.join(model_path, 'robot.sdf')

    #xacroでsdfファイルをurdfに変換
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': doc.toxml()}]
    ) # type: ignore

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    #ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(model_path, "world.sdf")

    #ignition gazeboの起動設定
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 
            'launch', 
            'ign_gazebo.launch.py')]),
        launch_arguments=[('ign_args', [' -r -v 3 ' + world])]
    )
    
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        robot_state_publisher,
        joint_state_pub_node,
        bridge_ign2ros2,
        base_link2base_footprint_tf,
        lidar_box2gpu_lidar_tf,
        teleop,
        ign_gz,
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),
    ])
