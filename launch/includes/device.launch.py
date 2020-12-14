from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def arg(name, value):
    return DeclareLaunchArgument(
        name,
        default_value=value
    )

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manager'),
        DeclareLaunchArgument('device_id'),
        DeclareLaunchArgument('bootorder'),
        DeclareLaunchArgument('devnums'),
        DeclareLaunchArgument('rgb_frame_id'),
        DeclareLaunchArgument('depth_frame_id'),
        DeclareLaunchArgument('rgb_camera_info_url'),
        DeclareLaunchArgument('depth_camera_info_url'),
        arg('depth_registration', 'true'),
        arg('color_depth_synchronization', 'false'),
        arg('auto_exposure', 'true'),
        arg('auto_white_balance', 'true'),
        arg('respawn', 'false'),
        arg('rgb', 'rgb'),
        arg('ir', 'ir'),
        arg('depth', 'depth'),
        arg('depth_registered', 'depth_registered'),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='astra_camera',
                    name='driver',
                    plugin='astra_camera::AstraDriverNode',
                    parameters=[
                        {'device_id': LaunchConfiguration('device_id')},
                        {'bootorder': LaunchConfiguration('bootorder')},
                        {'devnums': LaunchConfiguration('devnums')},
                        {'rgb_camera_info_url': LaunchConfiguration('rgb_camera_info_url')},
                        {'depth_camera_info_url': LaunchConfiguration('depth_camera_info_url')},
                        {'rgb_frame_id': LaunchConfiguration('rgb_frame_id')},
                        {'depth_frame_id': LaunchConfiguration('depth_frame_id')},
                        {'depth_registration': LaunchConfiguration('depth_registration')},
                        {'color_depth_synchronization': LaunchConfiguration('color_depth_synchronization')},
                        {'auto_exposure': LaunchConfiguration('auto_exposure')},
                        {'auto_white_balance': LaunchConfiguration('auto_white_balance')}
                    ],
                    remappings=[
                        ("ir", LaunchConfiguration('ir')),
                        ('rgb', LaunchConfiguration('rgb')),
                        ('depth', LaunchConfiguration('depth')),
                        ('depth_registered', LaunchConfiguration('depth_registered')),
                        ([LaunchConfiguration('rgb'), '/image'], [LaunchConfiguration('rgb'), '/image_raw']),
                        ([LaunchConfiguration('depth'), '/image'], [LaunchConfiguration('depth_registered'), '/image_raw'])
                    ]
                )
            ]
        )
    ])