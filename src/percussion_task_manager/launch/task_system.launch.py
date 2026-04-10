from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        # --- Perception ---
        DeclareLaunchArgument('marker_size',        default_value='0.0398',      description='ArUco marker side length in metres'),

        # --- Task manager ---
        DeclareLaunchArgument('capture_timeout_sec', default_value='10.0',       description='Capture service timeout in seconds'),

        # --- Motion ---
        DeclareLaunchArgument('robot_ip',           default_value='169.254.0.22', description='UR10e RTDE IP address'),
        DeclareLaunchArgument('default_velocity',   default_value='0.2',          description='Default joint velocity (m/s)'),
        DeclareLaunchArgument('default_accel',      default_value='0.2',          description='Default joint acceleration (m/s²)'),
        DeclareLaunchArgument('contact_force',      default_value='5.0',          description='Contact detection force threshold (N)'),
        DeclareLaunchArgument('contact_timeout',    default_value='5.0',          description='Contact detection timeout (s)'),
    ]

    nodes = [
        Node(
            package='percussion_perception',
            executable='capture_service_node',
            name='capture_service_node',
            output='screen',
            parameters=[{
                'markerSize': LaunchConfiguration('marker_size'),
            }],
        ),
        Node(
            package='percussion_task_manager',
            executable='task_manager_node',
            name='task_manager',
            output='screen',
            parameters=[{
                'capture_timeout_sec': LaunchConfiguration('capture_timeout_sec'),
            }],
        ),
        Node(
            package='percussion_motion',
            executable='percussion_motion_node',
            name='percussion_motion',
            output='screen',
            parameters=[{
                'robot_ip':         LaunchConfiguration('robot_ip'),
                'default_velocity': LaunchConfiguration('default_velocity'),
                'default_accel':    LaunchConfiguration('default_accel'),
                'contact_force':    LaunchConfiguration('contact_force'),
                'contact_timeout':  LaunchConfiguration('contact_timeout'),
            }],
        ),
        Node(  # Broadcast transform from camera (child) to TCP/tool0 (parent)
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool0_to_camera_broadcaster',
            arguments=['0.0', '-0.12775', '0.04451', '3.14', '0.0', '0.0', 'tool0', 'camera_frame'],
            output='screen',
        ),
        Node(  # Broadcast transform to rotate robot base 45° about x-axis
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0.785398', 'world', 'base_link'],
            output='screen',
        ),
    ]

    return LaunchDescription(args + nodes)
