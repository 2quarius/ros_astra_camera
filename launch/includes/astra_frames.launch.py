from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def arg(name, value):
    return DeclareLaunchArgument(
        name,
        default_value=value
    )

def generate_launch_description():
    optical_rotate = ['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966']
    return LaunchDescription([
        arg('camera', 'camera'),
        arg('tf_prefix', ''),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('camera'), '_base_link'],
            arguments=['0', '-0.02', '0', '0', '0', '0', 
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_link'], 
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_depth_frame']
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('camera'), '_base_link1'],
            arguments=['0', '-0.045', '0', '0', '0', '0',
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_link'],
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_rgb_frame']
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('camera'), '_base_link2'],
            arguments=optical_rotate + [
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_depth_frame'],
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_depth_optical_frame']
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('camera'), '_base_link3'],
            arguments=optical_rotate + [
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_rgb_frame'],
                       [LaunchConfiguration('tf_prefix'), '/', LaunchConfiguration('camera'), '_rgb_optical_frame']
            ]
        )
    ])