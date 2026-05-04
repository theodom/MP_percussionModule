from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:

    args = [
        # --- Perception ---
        DeclareLaunchArgument('marker_size',        default_value='0.0398',       description='ArUco marker side length in metres'),

        # --- Motion ---
        DeclareLaunchArgument('robot_ip',           default_value='169.254.0.22', description='UR10e IP address'),
        DeclareLaunchArgument('default_velocity',   default_value='1.0',          description='Default joint velocity (m/s)'),
        DeclareLaunchArgument('default_accel',      default_value='0.50',         description='Default joint acceleration (m/s²)'),
        DeclareLaunchArgument('contact_force',      default_value='5.0',          description='Contact detection force threshold (N)'),
        DeclareLaunchArgument('contact_timeout',    default_value='5.0',          description='Contact detection timeout (s)'),

        # --- Arduino ---
        DeclareLaunchArgument('arduino_port',       default_value='/dev/ttyACM0', description='Arduino serial port'),
        DeclareLaunchArgument('arduino_baudrate',   default_value='115200',       description='Arduino serial baudrate'),
    ]

    nodes = GroupAction([
        PushRosNamespace('percussion'),

        Node(
            package='percussion_task_manager',
            executable='task_manager_node',
            name='node',
            namespace='task_manager',
            output='screen',
            parameters=[],
        ),
        Node(
            package='perception',
            executable='perception_node',
            name='node',
            namespace='perception',
            output='screen',
            parameters=[{
                'markerSize': LaunchConfiguration('marker_size'),
            }],
        ),
        Node(
            package='motion',
            executable='motion_node',
            name='node',
            namespace='motion',
            output='screen',
            parameters=[{
                'robot_ip':         LaunchConfiguration('robot_ip'),
                'default_velocity': LaunchConfiguration('default_velocity'),
                'default_accel':    LaunchConfiguration('default_accel'),
                'contact_force':    LaunchConfiguration('contact_force'),
                'contact_timeout':  LaunchConfiguration('contact_timeout'),
            }],
        ),
        Node(
            package='percussion_arduino_bridge',
            executable='arduino_bridge_node',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'port':     LaunchConfiguration('arduino_port'),
                'baudrate': LaunchConfiguration('arduino_baudrate'),
            }],
        ),

        # TF broadcasters are kept outside percussion/ — tf2 topics (/tf, /tf_static)
        # are always global and must NOT be namespaced or TF lookups will break.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool0_to_camera_broadcaster',
            output='screen',
            arguments=['0.0', '-0.12775', '0.04451', '3.14', '0.0', '0.0', 'tool0', 'camera_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0.785398', 'world', 'base_link'],
        ),
    ])

    return LaunchDescription(args + [nodes])