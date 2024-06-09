from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
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
                ('/model/f1d2/odometry', '/odometry'),
                ('/model/f1d2/tf', '/tf')
            ],
        )
    ])
