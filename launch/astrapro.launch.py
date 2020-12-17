import launch
import launch.actions
import launch.substitutions
import launch.conditions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.actions import SetLaunchConfiguration

def arg(name, value):
    return launch.actions.DeclareLaunchArgument(
        name,
        default_value=value
    )

def generate_launch_description():
    rgbd_launch_dir = get_package_share_directory('rgbd_launch')
    astra_camera_dir = get_package_share_directory('astra_camera')
    return launch.LaunchDescription([
        arg('camera', 'camera'),
        arg('rgb_frame_id', [launch.substitutions.LaunchConfiguration('camera'), '_rgb_optical_frame']),
        arg('depth_frame_id', [launch.substitutions.LaunchConfiguration('camera'), '_depth_optical_frame']),
        arg('device_id', "#1"),
        arg('bootorder', '0'),
        arg('devnums', '1'),
        arg('rgb_camera_info_url', ""),
        arg("depth_camera_info_url", ""),
        arg("depth_registration", 'true'),
        arg('color_depth_synchronization', 'false'),
        arg('auto_exposure', 'true'),
        arg('auto_white_balance', 'true'),
        arg('rgb', "rgb"),
        arg('ir', 'ir'),
        arg('depth', 'depth'),
        arg('load_driver', 'true'),
        arg('publish_tf', 'true'),
        arg('rgb_processing', 'true'),
        arg('debayer_processing', 'false'),
        arg('ir_processing', 'false'),
        arg('depth_processing', 'true'),
        arg('depth_registered_processing', 'true'),
        arg('disparity_processing', 'false'),
        arg('disparity_registered_processing', 'false'),
        arg('hw_registered_processing', 'true'),
        arg('sw_registered_processing', 'false'),
        arg('respawn', 'false'),
        arg('num_worker_threads', '4'),
        arg('index', '0'),
        launch.actions.DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='component_container',
            namespace='',
        ),
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value='component_container'
        ),
        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(launch.substitutions.LaunchConfiguration('camera')),
            launch_ros.actions.Node(
                package='astra_camera',
                executable='camera_node',
                name=[launch.substitutions.LaunchConfiguration('camera'), '_rgb'],
                parameters=[
                    {"vendor": 0x2bc5},
                    {"product": 0x0502},
                    {"serial": 'SN0001'},
                    {"index": launch.substitutions.LaunchConfiguration('index')},
                    {"width": 640},
                    {"height": 480},
                    {"video_mode": "yuyv"},
                    {"frame_rate": 30},
                    {"timestamp_method": "start"},
                    {"camera_info_url": ""}
                ],
                remappings=[
                    (['/', launch.substitutions.LaunchConfiguration('camera'), '/image_raw'], ['/', launch.substitutions.LaunchConfiguration('camera'), '/rgb/image_raw'])
                ]
            ),
            arg('manager', [launch.substitutions.LaunchConfiguration('camera'), '_nodelet_manager']),
            arg('debug', 'false'),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(astra_camera_dir + '/launch/includes/device.launch.py'),
                launch_arguments = [
                    ('container', launch.substitutions.LaunchConfiguration('container')),
                    ("manager", launch.substitutions.LaunchConfiguration('manager')),
                    ("device_id", launch.substitutions.LaunchConfiguration('device_id')),
                    ("bootorder", launch.substitutions.LaunchConfiguration('bootorder')),
                    ("devnums", launch.substitutions.LaunchConfiguration('devnums')),
                    ("rgb_frame_id", launch.substitutions.LaunchConfiguration('rgb_frame_id')),
                    ("depth_frame_id", launch.substitutions.LaunchConfiguration('depth_frame_id')),
                    ("rgb_camera_info_url", launch.substitutions.LaunchConfiguration('rgb_camera_info_url')),
                    ("depth_camera_info_url", launch.substitutions.LaunchConfiguration('depth_camera_info_url')),
                    ("rgb", launch.substitutions.LaunchConfiguration('rgb')),
                    ("ir", launch.substitutions.LaunchConfiguration('ir')),
                    ("depth", launch.substitutions.LaunchConfiguration('depth')),
                    ("respawn", launch.substitutions.LaunchConfiguration('respawn')),
                    ("depth_registration", launch.substitutions.LaunchConfiguration('depth_registration')),
                    ("color_depth_synchronization", launch.substitutions.LaunchConfiguration('color_depth_synchronization')),
                    ("auto_exposure", launch.substitutions.LaunchConfiguration('auto_exposure')),
                    ("auto_white_balance", launch.substitutions.LaunchConfiguration('auto_white_balance'))
                ],
                condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('load_driver'))
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/processing.launch.py'),
                launch_arguments=[
                    ('container', launch.substitutions.LaunchConfiguration('container')),
                    ("manager", launch.substitutions.LaunchConfiguration('manager')),
                    ("rgb", launch.substitutions.LaunchConfiguration('rgb')),
                    ("ir", launch.substitutions.LaunchConfiguration('ir')),
                    ("depth", launch.substitutions.LaunchConfiguration('depth')),
                    ("respawn", launch.substitutions.LaunchConfiguration('respawn')),
                    ("rgb_processing", launch.substitutions.LaunchConfiguration('rgb_processing')),
                    ("debayer_processing", launch.substitutions.LaunchConfiguration('debayer_processing')),
                    ("ir_processing", launch.substitutions.LaunchConfiguration('ir_processing')),
                    ("depth_processing", launch.substitutions.LaunchConfiguration('depth_processing')),
                    ("depth_registered_processing", launch.substitutions.LaunchConfiguration('depth_registered_processing')),
                    ("disparity_processing", launch.substitutions.LaunchConfiguration('disparity_processing')),
                    ("disparity_registered_processing", launch.substitutions.LaunchConfiguration('disparity_registered_processing')),
                    ("hw_registered_processing", launch.substitutions.LaunchConfiguration('hw_registered_processing')),
                    ("sw_registered_processing", launch.substitutions.LaunchConfiguration('sw_registered_processing'))
                ]
            )
        ], scoped=False),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(astra_camera_dir + '/launch/includes/astra_frames.launch.py'),
            launch_arguments=[
                ("camera", launch.substitutions.LaunchConfiguration("camera"))
            ],
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('publish_tf'))
        )
    ])