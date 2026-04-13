from enum import Enum
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
from percussion_interfaces.srv import TriggerCapture
from percussion_interfaces.action import ExecuteMotion
from percussion_interfaces.msg import Pose6D


class TaskState(str, Enum):
    IDLE                = 'IDLE'
    TASK_REQUESTED      = 'TASK_REQUESTED'
    CAPTURING           = 'CAPTURING'
    POSE_ACQUIRED       = 'POSE_ACQUIRED'
    MOVING_TO_WEDGELOCK = 'MOVING_TO_WEDGELOCK'
    HAMMERING           = 'HAMMERING'
    DONE                = 'DONE'
    RETURNING           = 'RETURNING'
    ERROR               = 'ERROR'


def _make_pose6d(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0) -> Pose6D:
    p = Pose6D()
    p.x, p.y, p.z = x, y, z
    p.rx, p.ry, p.rz = rx, ry, rz
    return p


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        # Parameters 
        self.declare_parameter('capture_service_name', '/trigger_capture')
        self.declare_parameter('target_frame', 'base')

        capture_service_name = self.get_parameter('capture_service_name').get_parameter_value().string_value
        self._target_frame   = self.get_parameter('target_frame').get_parameter_value().string_value

        # Services/Topics
        self._start_srv     = self.create_service(Trigger, '/start_task', self.start_task_callback)
        self._state_pub     = self.create_publisher(String, '/task_manager/state', 10)
        self._capture_client: Client = self.create_client(TriggerCapture, capture_service_name)
        self._motion_client = ActionClient(self, ExecuteMotion, '/execute_motion')

        self._current_state = TaskState.IDLE
        self._pending_capture_call = None
        self._returning = False
        self._sequence: List[dict] = []

        self.publish_state(self._current_state)
        self.get_logger().info('Task manager node started.')

    # ------------------------------------------------------------------
    # Sequence definition
    # ------------------------------------------------------------------

    def _build_sequence(self, marker_pose: Pose6D) -> List[dict]:
        """
        Define the full motion sequence for one percussion task.
        Each step is a dict with keys: motion_type, marker_pose, approach_offset.
        Edit here to add, remove, or reorder steps.
        """
        return [
            {
                'motion_type':    'MOVE_TO_MARKER', # 10 cm standoff in base X
                'marker_pose':    marker_pose,
                'approach_offset': [-0.07, 0.0, 0.0, 0.0, 0.0, 0.0],  # Base Frame
            },
            {
                'motion_type':    'MOVE_TO_CONTACT', # Touch Ledger facing marker
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.010, 0.0, 0.0, 0.0, 0.0, 0.0],   # Base frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # move backwards from Ledger + UP
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0, 0.12, -0.05, 0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Rotate Tool to face wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, -1.5701, 0.0],   # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Move towards wedgelock (sideways) for contact 2D
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.080, 0, 0, 0, 0.0, 0.0],  # TCP frame
            },
            {
                'motion_type': 'MOVE_TO_CONTACT', # Touch ledger top down
                'marker_pose': _make_pose6d(),
                'approach_offset': [0.0, 0.00707, -0.00707, 0.0, 0.0, 0.0], # Base Frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE up for clearance
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0,0.06, 0, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE closer to pole 
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.025, 0.0, 0.10, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'MOVE_TO_CONTACT', # Touch bar sideways
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, -0.00707, -0.00707, 0.0, 0.0, 0.0], # Base Frame            
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Move into striking position
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, -0.00300, 0.0, 0.0, 0.0],   # TCP frame
            },
        ]

    def _build_return_sequence(self) -> List[dict]:
        """
        Sequence executed automatically after the main task completes.
        Edit here to adjust the return/retract motion before going home.
        """
        return [
            {
                'motion_type':    'RELATIVE_MOVE', # Retract from wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.05, -0.10, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Rotate Tool to unface wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, 1.5701, 0.0],   # TCP frame
            },
            {
                'motion_type':    'RETURN_HOME',
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ]

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def publish_state(self, state: TaskState) -> None:
        self._current_state = state
        msg = String()
        msg.data = state.value
        self._state_pub.publish(msg)
        self.get_logger().info(f'State -> {state.value}')

    # ------------------------------------------------------------------
    # /start_task service
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Capture callback
    # ------------------------------------------------------------------

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
            return

        for det in result.detections:
            self.get_logger().info(
                'Detected marker:  id=%d : x=%.4f, y=%.4f, z=%.4f'
                % (det.marker_id, det.pose.x, det.pose.y, det.pose.z)
            )

        selected = result.detections[0]
        self.get_logger().info(
            'Selected marker: id=%d : x=%.4f, y=%.4f, z=%.4f'
            % (selected.marker_id, selected.pose.x, selected.pose.y, selected.pose.z)
        )


        self.publish_state(TaskState.POSE_ACQUIRED)

        # Rework to be more general motion.


        self._sequence = self._build_sequence(selected.pose)
        self._returning = False
        self._execute_next_step()

    # ------------------------------------------------------------------
    # Sequence execution
    # ------------------------------------------------------------------

    def _execute_next_step(self) -> None:
        if not self._sequence:
            self.get_logger().info(f'if not self.sequence')
            if not self._returning:
                # Main sequence done — start return sequence
                self._returning = True
                self._sequence = self._build_return_sequence()
                self.publish_state(TaskState.HAMMERING)
                self.get_logger().info(f'Hammer sequence . . . ')
                self.publish_state(TaskState.DONE)
            else:
                # Return sequence done — back to idle
                self.publish_state(TaskState.IDLE)
                return

        step = self._sequence.pop(0)
        phase = 'RETURNING' if self._returning else 'MOVING_TO_WEDGELOCK'
        self.get_logger().info(
            f'{phase} step: {step["motion_type"]} '
            f'({len(self._sequence)} steps remaining)'
        )
        self._send_motion_goal(step)

    def _send_motion_goal(self, step: dict) -> None:
        if not self._motion_client.server_is_ready():
            self.get_logger().error('Motion action server not available.')
            self.publish_state(TaskState.ERROR)
            return

        goal = ExecuteMotion.Goal()
        goal.motion_type     = step['motion_type']
        goal.marker_pose     = step['marker_pose']
        goal.approach_offset = step['approach_offset']

        state = TaskState.RETURNING if self._returning else TaskState.MOVING_TO_WEDGELOCK
        self.publish_state(state)
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
            self.get_logger().info('Step complete.')
            self._execute_next_step()
        else:
            self.get_logger().error(f'Step failed: {result.message}')
            self._sequence.clear()
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
