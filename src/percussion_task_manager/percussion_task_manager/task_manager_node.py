from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
from percussion_interfaces.srv import TriggerCapture
from percussion_interfaces.action import ExecuteMotion


class TaskState(str, Enum):
    IDLE           = 'IDLE'
    CAPTURING      = 'CAPTURING'
    POSE_ACQUIRED  = 'POSE_ACQUIRED'
    MOVE_TO_MARKER = 'MOVE_TO_MARKER'
    DONE           = 'DONE'
    ERROR          = 'ERROR'


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        self.declare_parameter('capture_service_name', '/trigger_capture')
        self.declare_parameter('target_frame', 'base')
        self.declare_parameter('capture_timeout_sec', 5.0)

        capture_service_name = self.get_parameter('capture_service_name').get_parameter_value().string_value
        self._target_frame   = self.get_parameter('target_frame').get_parameter_value().string_value
        self._capture_timeout = self.get_parameter('capture_timeout_sec').get_parameter_value().double_value

        self._start_srv      = self.create_service(Trigger, '/start_task', self.start_task_callback)
        self._state_pub      = self.create_publisher(String, '/task_manager/state', 10)
        self._capture_client: Client = self.create_client(TriggerCapture, capture_service_name)
        self._motion_client  = ActionClient(self, ExecuteMotion, '/execute_motion')

        self._current_state = TaskState.IDLE
        self._pending_capture_call = None

        self.publish_state(self._current_state)
        self.get_logger().info('Task manager node started.')

    def publish_state(self, state: TaskState) -> None:
        self._current_state = state
        msg = String()
        msg.data = state.value
        self._state_pub.publish(msg)
        self.get_logger().info(f'State -> {state.value}')

    def start_task_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self._current_state not in [TaskState.IDLE, TaskState.DONE, TaskState.ERROR]:
            response.success = False
            response.message = f'Task manager busy, current state: {self._current_state.value}'
            return response

        if not self._capture_client.service_is_ready():
            self.get_logger().warn('Capture service not reported ready yet; sending request anyway.')

        self.publish_state(TaskState.CAPTURING)
        self._pending_capture_call = self._capture_client.call_async(TriggerCapture.Request())
        self._pending_capture_call.add_done_callback(self._on_capture_done)

        response.success = True
        response.message = 'Task accepted; capture request sent.'
        return response

    def _on_capture_done(self, future) -> None:
        try:
            result: Optional[TriggerCapture.Response] = future.result()
        except Exception as exc:
            self.publish_state(TaskState.ERROR)
            self.get_logger().error(f'Capture service call failed: {exc}')
            return

        if result is None:
            self.publish_state(TaskState.ERROR)
            self.get_logger().error('Capture service returned no result.')
            return

        if not result.success:
            self.publish_state(TaskState.ERROR)
            self.get_logger().warning(f'Capture failed: {result.message}')
            return

        if len(result.detections) == 0:
            self.publish_state(TaskState.IDLE)
            self.get_logger().warning('No markers detected.')
            # TO DO: rotate camera -> try again.
            return

        for det in result.detections:
            self.get_logger().info(
                'Detected marker:  id=%d : x=%.4f, y=%.4f, z=%.4f'
                % (det.marker_id, det.pose.x, det.pose.y, det.pose.z)
            )

        # evt. vervangen door laagste marker ID, of opvolgend van laatst geklopte marker id,...
        selected = result.detections[0]  # Marker pose in base frame

        self.get_logger().info(
            'Selected marker: id=%d : x=%.4f, y=%.4f, z=%.4f'
            % (selected.marker_id, selected.pose.x, selected.pose.y, selected.pose.z)
        )
        self.publish_state(TaskState.POSE_ACQUIRED)
        self._send_motion_goal(selected.pose)

    def _send_motion_goal(self, pose) -> None:
        if not self._motion_client.server_is_ready():
            self.get_logger().error('Motion action server not available.')
            self.publish_state(TaskState.ERROR)
            return

        goal = ExecuteMotion.Goal()
        goal.motion_type     = 'MOVE_TO_MARKER'
        goal.marker_pose     = pose
        goal.approach_offset = [-0.10, 0.0, 0.0, 0.0, 0.0, 0.0]  # -10 cm in TCP x

        self.publish_state(TaskState.MOVE_TO_MARKER)
        send_future = self._motion_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_motion_goal_accepted)

    def _on_motion_goal_accepted(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion goal rejected by motion node.')
            self.publish_state(TaskState.ERROR)
            return
        goal_handle.get_result_async().add_done_callback(self._on_motion_result)

    def _on_motion_result(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info('Motion complete.')
            self.publish_state(TaskState.DONE)
        else:
            self.get_logger().error(f'Motion failed: {result.message}')
            self.publish_state(TaskState.ERROR)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
