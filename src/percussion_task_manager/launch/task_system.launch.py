from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node( # Perception node, realsense camera
            package='percussion_perception',
            executable='capture_service_node',
            name='capture_service_node',
            output='screen',
            arguments=[
                {'markerSize': 0.0398}
            ],
        ),
        Node( # Task manager node = orchestrator, central node
            package='percussion_task_manager',
            executable='task_manager_node',
            name='task_manager',
            output='screen',
            parameters=[
                {
                    'capture_timeout_sec': 10.0
                }
            ],
        ),
        Node( # Motion node, controls UR10e via RTDE
            package='percussion_motion',
            executable='percussion_motion_node',
            name='percussion_motion',
            output='screen',
            parameters=[
                {
                    'robot_ip':         '169.254.0.22',
                    'default_velocity': 0.2,
                    'default_accel':    0.2,
                    'contact_force':    5.0,
                    'contact_timeout':  5.0,
                }
            ],
        ),
        Node( # Broadcast transform from camera (child) to TCP/tool0 (parent)
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tool0_to_camera_broadcaster',
        # positional order: x y z yaw pitch roll frame_id child_frame_id
        arguments=['0.0', '-0.12775', '0.04451', '3.14', '0.0', '0.0', 'tool0', 'camera_frame'],
        output='screen',
        ),
        Node( # Broadcast transform to rotate robot base 45° about x-axis.
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_broadcaster',
        arguments=[
            '0', '0', '0',          # x y z
            '0', '0', '0.785398',   # yaw pitch roll
            'world',
            'base_link',
        ],
        output='screen',
        )
    ])
