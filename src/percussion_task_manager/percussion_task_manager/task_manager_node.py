from enum import Enum
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
from percussion_interfaces.srv import TriggerCapture
from percussion_interfaces.action import ExecuteMotion, ArduinoCommand
from percussion_interfaces.msg import Pose6D


class TaskState(str, Enum):
    IDLE                = 'IDLE'
    TASK_REQUESTED      = 'TASK_REQUESTED'
    CAPTURING           = 'CAPTURING'
    POSE_ACQUIRED       = 'POSE_ACQUIRED'
    MOVING_TO_WEDGELOCK = 'MOVING_TO_WEDGELOCK'
    AT_MARKER           = 'AT_MARKER'
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
        self._state_sub     = self.create_subscription(String, '/task_manager/state', self._on_state_changed, 10)
        self._capture_client: Client = self.create_client(TriggerCapture, capture_service_name)
        self._motion_client = ActionClient(self, ExecuteMotion, '/execute_motion')
        self._arduino_client = ActionClient(self, ArduinoCommand, '/arduino_command')

        self._selected_marker: Optional[Pose6D] = None
        self._current_state = TaskState.IDLE
        self._pending_capture_call = None
        self._on_sequence_done = None
        self._sequence: List[dict] = []

        self.publish_state(self._current_state)
        self.get_logger().info('Task manager node started.')

    # ------------------------------------------------------------------
    # Sequence definition
    # ------------------------------------------------------------------

    def _build_sequence(self, marker_pose: Pose6D) -> List[dict]:
        """
        Define the full motion sequence for one percussion task.
        Each step is a dict with keys: motion_type, marker_pose, approach_offset, contact_force.
        contact_force is optional and defaults to ROS parameter if not specified.
        Edit here to add, remove, or reorder steps.
        """
        return [
            {
                'motion_type':    'MOVE_TO_MARKER', # Move to Marker with approach_offset
                'marker_pose':    marker_pose,
                'approach_offset': [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],  # Base Frame
            },
            {
                'motion_type':    'MOVE_TO_CONTACT', # Touch Ledger facing marker
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.020, 0.0, 0.0, 0.0, 0.0, 0.0],   # Base frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # move backwards from Ledger + UP
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0, 0.15, -0.05, 0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Rotate Tool to face wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, -1.5701, 0.0],   # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Move towards wedgelock (sideways) before contact 2D
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.080, -0.04, 0, 0, 0.0, 0.0],  # TCP frame
            },
            {
                'motion_type': 'MOVE_TO_CONTACT', # Touch ledger top down (contact 2D)
                'marker_pose': _make_pose6d(),
                'approach_offset': [0.0, 0.00707, -0.00707, 0.0, 0.0, 0.0], # Base Frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE up for clearance
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0, 0.08, 0, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE closer to wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.045, 0.0, 0.11, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'MOVE_TO_CONTACT', # Touch bar sideways
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, -0.00707, -0.00707, 0.0, 0.0, 0.0], # Base Frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE back from bar
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, -0.06, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE down ready for wedgelock insert
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, -0.05, 0.0, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE wedgelock in position
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.015, 0.045, 0.0, 0.0, 0.0], # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Move into striking position
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.010, 0.0, 0.0, 0.0, 0.0],   # TCP frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # MOVE over wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, -0.00, 0.015, 0.0, 0.0, 0.0], # TCP Frame
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Touch bar sideways
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, -0.007, 0.0, 0.0, 0.0, 0.0], # TCP Frame
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
                'contact_force':   5.0,
            },
            {
                'motion_type':    'RELATIVE_MOVE', # Retract from wedgelock
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, 1.57, 0.0], # TCP frame
                'contact_force':   5.0,
            },
            {
                'motion_type':    'RETURN_HOME',
                'marker_pose':    _make_pose6d(),
                'approach_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'contact_force':   5.0,
            },
        ]

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def _on_state_changed(self, _msg: String) -> None:
        state = _msg.data

        match state:
            case TaskState.TASK_REQUESTED: 
                self.run_capture_service()
            
            case TaskState.CAPTURING:

                pass
            case TaskState.POSE_ACQUIRED:
                # ENKEL VOOR THUIS TESTEN... ANDERS NIET GEBRUIKEN!
                # self._selected_marker = _make_pose6d(0.6597723446915864, 0.4272507575618981, 0.486163269104955, 1.6956424868194673, 0.7053985046295215, 1.8109614421292222)
                if self._selected_marker is None:
                    self.get_logger().error('POSE_ACQUIRED but no marker available')
                    self.publish_state(TaskState.ERROR)
                    return

                self._sequence = self._build_sequence(self._selected_marker)
                self._on_sequence_done = lambda: self.publish_state(TaskState.AT_MARKER)
                self._execute_next_step()
            case TaskState.AT_MARKER:
                self.publish_state(TaskState.HAMMERING)
                self._send_arduino_command('HAMMER_REQ', '5', 'a',
                                           on_success=TaskState.DONE,
                                           on_failure=TaskState.ERROR)
            case TaskState.HAMMERING:
                # waiting for hammer action result (handled by callback)
                pass
            case TaskState.DONE:
                # Transition logic will go here
                self.publish_state(TaskState.RETURNING)
                pass
            case TaskState.RETURNING:
                # Transition logic will go here
                self._sequence = self._build_return_sequence()
                self._on_sequence_done = lambda: self.publish_state(TaskState.IDLE)
                self._execute_next_step()
            case _:
        
                pass

    



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
        self.publish_state(TaskState.TASK_REQUESTED)
        response.success = True
        response.message = 'Task requested.'
        return response
    
    # ------------------------------------------------------------------
    # Start capture service
    # ------------------------------------------------------------------

    def run_capture_service(self):
        if not self._capture_client.service_is_ready():
            self.get_logger().warn('Capture service not reported ready yet; sending request anyway.')

        self._selected_marker = None
        self._pending_capture_call = self._capture_client.call_async(TriggerCapture.Request())
        self._pending_capture_call.add_done_callback(self._on_capture_done)
        self.publish_state(TaskState.CAPTURING)



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

        self._selected_marker = selected.pose
        self.publish_state(TaskState.POSE_ACQUIRED)

    # ------------------------------------------------------------------
    # Sequence execution
    # ------------------------------------------------------------------

    def _execute_next_step(self) -> bool:
        if not self._sequence:
            if self._on_sequence_done:
                self._on_sequence_done()
            return True

        step = self._sequence.pop(0)
        self.get_logger().info(
            f'Step: {step["motion_type"]} ({len(self._sequence)} steps remaining)'
        )
        self._send_motion_goal(step)
        return False

    def _send_motion_goal(self, step: dict) -> None:
        if step['marker_pose'] is None:
            self.get_logger().error('Attempting to send goal with None marker_pose')
            self.publish_state(TaskState.ERROR)
            return

        if not self._motion_client.server_is_ready():
            self.get_logger().error('Motion action server not available.')
            self.publish_state(TaskState.ERROR)
            return

        goal = ExecuteMotion.Goal()
        goal.motion_type     = step['motion_type']
        goal.marker_pose     = step['marker_pose']
        goal.approach_offset = step['approach_offset']
        goal.contact_force   = step.get('contact_force', 2.0)  # Default to 2.0 N if not specified

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                # self.get_logger().error('Motion goal rejected by motion node.')
                self.publish_state(TaskState.ERROR)
                return
            handle.get_result_async().add_done_callback(self._on_motion_result)

        self._motion_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    def _on_motion_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Motion action failed (server may have crashed): {exc}')
            self._sequence.clear()
            self._on_sequence_done = None
            self.publish_state(TaskState.ERROR)
            return
        if result.success:
            self.get_logger().info('Step complete.')
            self._execute_next_step()
        else:
            self.get_logger().error(f'Step failed: {result.message}')
            self._sequence.clear()
            self._on_sequence_done = None
            self.publish_state(TaskState.ERROR)

    # ------------------------------------------------------------------
    # Arduino command action
    # ------------------------------------------------------------------

    def _send_arduino_command(self, msg_type: str, data: str, msg_info: str,
                              on_success, on_failure) -> None:
        """
        Send a generic ArduinoCommand action goal.
        on_success / on_failure: TaskState to publish, or callable to invoke.
        """
        if not self._arduino_client.server_is_ready():
            self.get_logger().error('Arduino action server not available.')
            self.publish_state(TaskState.ERROR)
            return

        goal = ArduinoCommand.Goal()
        goal.msg_type = msg_type
        goal.data     = data
        goal.msg_info = msg_info

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f'{msg_type} goal rejected.')
                self.publish_state(TaskState.ERROR)
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            try:
                result = future.result().result
            except Exception as exc:
                self.get_logger().error(f'{msg_type} action failed: {exc}')
                self.publish_state(TaskState.ERROR)
                return
            if result.success:
                if callable(on_success):
                    on_success()
                else:
                    self.publish_state(on_success)
            else:
                self.get_logger().error(f'{msg_type} failed: {result.message}')
                if callable(on_failure):
                    on_failure()
                else:
                    self.publish_state(on_failure)

        self._arduino_client.send_goal_async(goal).add_done_callback(_on_goal_response)


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
