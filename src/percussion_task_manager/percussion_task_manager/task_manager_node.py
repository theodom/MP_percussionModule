from enum import Enum
from typing import Optional, List
import json

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from percussion_interfaces.srv import TriggerCapture, StartTask
from percussion_interfaces.action import ExecuteMotion, ArduinoCommand
from percussion_interfaces.msg import Pose6D, SystemState
from .task_modes import MODES
from .error_handler import ErrorHandler


class TaskState(str, Enum):
    IDLE                = 'IDLE'
    TASK_REQUESTED      = 'TASK_REQUESTED'
    AT_HOME             = 'AT_HOME'
    CAPTURING           = 'CAPTURING'
    POSE_ACQUIRED       = 'POSE_ACQUIRED'
    MOVING_TO_WEDGELOCK = 'MOVING_TO_WEDGELOCK'
    AT_MARKER           = 'AT_MARKER'
    HAMMERING           = 'HAMMERING'
    DONE                = 'DONE'
    RETURNING           = 'RETURNING'
    ERROR               = 'ERROR'




class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        # Parameters 
        self.declare_parameter('target_frame', 'base')

        self._target_frame   = self.get_parameter('target_frame').get_parameter_value().string_value

        # Services/Topics
        self._start_srv     = self.create_service(StartTask, 'start_task', self.start_task_callback)
        self._state_pub     = self.create_publisher(SystemState, 'state', 10)
        self._state_sub     = self.create_subscription(SystemState, 'state', self._on_state_changed, 10)
        self._capture_client: Client = self.create_client(TriggerCapture, '/percussion/perception/trigger_capture')
        self._motion_client = ActionClient(self, ExecuteMotion, '/percussion/motion/execute_motion')
        self._arduino_client = ActionClient(self, ArduinoCommand, '/percussion/arduino_bridge/arduino_command')

        # Timers
        self._timer = self.create_timer(1.0, self._publish_system_state)

        self._selected_marker: Optional[Pose6D] = None
        self._current_state = TaskState.IDLE
        self._previous_state: str = ''         # last non-ERROR state, used by the error handler to resume
        self._last_processed_state: str = ''
        self._pending_capture_call = None
        self._on_sequence_done = None
        self._sequence: List[dict] = []
        self._mode = None
        self._pause_handler = None
        self._no_marker_retries: int = 0       # inline "retry once" budget for empty capture results

        # SystemState tracking fields
        self._inductive_states: List[int] = []
        self._latest_tcp_pose: Optional[Pose6D] = None
        self._error_message: str = ''
        self._detected_marker_id: int = -1

        # Structured error handler (classifies error_message + dispatches recovery)
        self._error_handler = ErrorHandler(self)

        self.publish_state(self._current_state)
        self.get_logger().info('Task manager node started.')


    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def _on_state_changed(self, msg: SystemState) -> None:
        new_state_value = msg.task_state

        # Only execute state machine logic if TaskState actually changed since
        # the last time the callback ran (periodic 1Hz publishes have same value)
        if new_state_value == self._last_processed_state:
            return
        self._last_processed_state = new_state_value

        state = new_state_value

        match state:
            case TaskState.TASK_REQUESTED:
                self._sequence = self._mode['build_home_sequence']()
                self._on_sequence_done = lambda: self.publish_state(TaskState.AT_HOME)
                self._execute_next_step()

            case TaskState.AT_HOME:
                self.run_capture_service()

            case TaskState.CAPTURING:
                pass
            case TaskState.POSE_ACQUIRED:

                # #--- Add a fixed marker position for at home testing 
                # faked_list = [0.00630962,  0.06311957,  0.40705869, -1.72622068,  2.50957927,  0.20299939]
                # self._selected_marker = Pose6D()
                # self._selected_marker.x  = faked_list[0]
                # self._selected_marker.y  = faked_list[1]
                # self._selected_marker.z  = faked_list[2]
                # self._selected_marker.rx = faked_list[3]
                # self._selected_marker.ry = faked_list[4]
                # self._selected_marker.rz = faked_list[5]
                # self.get_logger().info(f"marker: {self._selected_marker}")
                # # #----------------------------------------------------
                if self._selected_marker is None:
                    self._error_message = "POSE_ACQUIRED but no Marker available"
                    self.get_logger().error(self._error_message)
                    self.publish_state(TaskState.ERROR)
                    return
                if self._mode is None:
                    self._error_message = 'POSE_ACQUIRED but no task mode selected'
                    self.get_logger().error(self._error_message)
                    self.publish_state(TaskState.ERROR)
                    return
                marker_list = [self._selected_marker.x, self._selected_marker.y, self._selected_marker.z, self._selected_marker.rx, self._selected_marker.ry, self._selected_marker.rz]
                self._pause_handler = lambda step, resume: self._read_inductive(
                    2, resume, self._raise_inductive_error
                )
                self._sequence = self._mode['build_sequence'](marker_list)
                self._on_sequence_done = lambda: self.publish_state(TaskState.AT_MARKER)
                self._execute_next_step()
            case TaskState.AT_MARKER:
                self.publish_state(TaskState.HAMMERING)
            case TaskState.HAMMERING:
                cmd = self._mode['arduino']
                self._send_arduino_command(cmd['msg_type'], cmd['data'], cmd['msg_info'],
                                           on_success=TaskState.DONE,
                                           on_failure=TaskState.ERROR)
            case TaskState.DONE:
                self.publish_state(TaskState.RETURNING)
            case TaskState.RETURNING:
                self._sequence = self._mode['build_return_sequence']()
                self._on_sequence_done = lambda: self.publish_state(TaskState.IDLE)
                self._execute_next_step()
            case TaskState.IDLE:
                self._error_message = ''
                self._inductive_states = []
                self._detected_marker_id = -1
                self._no_marker_retries = 0
                self._error_handler.reset_retries()
            case TaskState.ERROR:
                self._error_handler.handle_error(
                    self._error_message,
                    context={
                        'previous_state': self._previous_state,
                        'mode': self._mode.get('name') if self._mode else None,
                    },
                )
            case _:
                pass

    



    def publish_state(self, state: TaskState) -> None:
        # Track the last non-ERROR state so the error handler can resume after
        # a transient fault by re-publishing the state that was in flight.
        if self._current_state != TaskState.ERROR:
            self._previous_state = self._current_state.value
        self._current_state = state
        self._publish_system_state()
        self.get_logger().info(f'State -> {state.value}')

    def _raise_inductive_error(self) -> None:
        """Pause-handler failure path: tag the error message so the structured
        handler can classify it as INDUCTIVE_FAULT, then transition to ERROR."""
        self._error_message = 'INDUCTIVE sensor 2 not detected'
        self.get_logger().error(self._error_message)
        self.publish_state(TaskState.ERROR)

    def _publish_system_state(self) -> None:
        msg = SystemState()
        msg.task_state = self._current_state.value
        msg.task_mode = self._mode.get('name', '') if self._mode else ''
        msg.inductive_states = self._inductive_states
        msg.tcp_pose = self._latest_tcp_pose if self._latest_tcp_pose else Pose6D()
        msg.error_message = self._error_message
        msg.detected_marker_id = self._detected_marker_id
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------
    # /start_task service
    # ------------------------------------------------------------------

    def start_task_callback(self, request: StartTask.Request, response: StartTask.Response) -> StartTask.Response:
        mode_key = request.mode.upper()
        if mode_key not in MODES:
            response.success = False
            response.message = f"Unknown mode '{mode_key}'. Valid: {list(MODES.keys())}"
            return response
        self._mode = MODES[mode_key]
        self.publish_state(TaskState.TASK_REQUESTED)
        response.success = True
        response.message = f'Task requested: {mode_key}'
        self.get_logger().info(f"MODE: {mode_key}")
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
            self._error_message = "CAPTURE ERROR - Capture service call failed " + str(exc)

            self.publish_state(TaskState.ERROR)
            self.get_logger().error(self._error_message)
            return

        if result is None:
            self._error_message = 'Capture service returned no result'
            self.publish_state(TaskState.ERROR)
            self.get_logger().error(self._error_message)
            return

        if not result.success:
            self._error_message = "CAPTURE RESULT == FAILED: " + result.message
            self.publish_state(TaskState.ERROR)
            self.get_logger().warning(self._error_message)
            return

        if len(result.detections) == 0:
            # Retry once via the mode's home sequence before escalating to ERROR.
            # The mode's home is a known-safe pose with good marker visibility.
            if self._mode is not None and self._no_marker_retries < 1:
                self._no_marker_retries += 1
                self.get_logger().warning(
                    'No markers detected — re-running home sequence and retrying capture '
                    f'(attempt {self._no_marker_retries}/1).'
                )
                self._sequence = self._mode['build_home_sequence']()
                self._pause_handler = None
                self._on_sequence_done = self.run_capture_service
                self._execute_next_step()
                return

            self._error_message = 'MARKER NOT FOUND in view'
            self.publish_state(TaskState.ERROR)
            self.get_logger().warning(self._error_message)
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
        self._detected_marker_id = selected.marker_id
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

        if step['motion_type'] == 'PAUSE':
            self.get_logger().info(
                f'Sequence paused ({len(self._sequence)} steps remaining)'
            )
            if self._pause_handler:
                self._pause_handler(step, self._execute_next_step)
            else:
                self.get_logger().info(f"Continuing sequence immediately.")
                self._execute_next_step()
            return False

        self.get_logger().info(
            f'Step: {step["motion_type"]} ({len(self._sequence) + 1} steps remaining)'
        )
        self._send_motion_goal(step)
        return False

    def _send_motion_goal(self, step: dict) -> None:
        if not self._motion_client.server_is_ready():
            self._error_message = 'Motion action server not available'
            self.get_logger().error(self._error_message)
            self.publish_state(TaskState.ERROR)
            return

        goal = ExecuteMotion.Goal()
        goal.motion_type     = step['motion_type']
        goal.step_data       = json.dumps(step)

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                # self.get_logger().error('Motion goal rejected by motion node.')
                self._error_message = "Motion not accepted"
                self.publish_state(TaskState.ERROR)
                return
            handle.get_result_async().add_done_callback(self._on_motion_result)

        self._motion_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    def _on_motion_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Motion action failed (server may have crashed): {exc}')
            self._error_message = f'MOTION server exception: {exc}'
            self._sequence.clear()
            self._on_sequence_done = None
            self.publish_state(TaskState.ERROR)
            return
        if result.success:
            self.get_logger().info('Step complete.')
            if result.final_tcp_pose:
                self._latest_tcp_pose = result.final_tcp_pose
            self._execute_next_step()
        else:
            self._error_message = "MOTION STEP FAILED: " + result.message
            self.publish_state(TaskState.ERROR)
            self.get_logger().error(self._error_message)
            self._sequence.clear()
            self._on_sequence_done = None

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
            self._error_message = 'ARDUINO action server not available'
            self.get_logger().error(self._error_message)
            self.publish_state(TaskState.ERROR)
            return

        goal = ArduinoCommand.Goal()
        goal.msg_type = msg_type
        goal.data     = data
        goal.msg_info = msg_info

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                self._error_message = f'ARDUINO: {msg_type} goal rejected'
                self.get_logger().error(self._error_message)
                self.publish_state(TaskState.ERROR)
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            try:
                result = future.result().result
            except Exception as exc:
                self._error_message = f"ARDUINO: {msg_type} action failed: " + str(exc)
                self.get_logger().error(self._error_message)
                self.publish_state(TaskState.ERROR)
                return
            if result.success:
                if callable(on_success):
                    on_success(result.message)
                else:
                    self.publish_state(on_success)
            else:
                self._error_message = f'ARDUINO: {msg_type} failed: {result.message}'
                self.get_logger().error(self._error_message)
                if callable(on_failure):
                    on_failure()
                else:
                    self.publish_state(on_failure)

        self._arduino_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    def _read_inductive(self, sensor: int, on_success, on_failure) -> None:
        """Read inductive sensor and invoke on_success/on_failure based on value."""
        def _check_sensor(msg: str):
            try:
                # Parse response: "IND_VALUES|DONE|<val_1>;<val_2>"
                values = [int(v) for v in msg.split(';')]
                self._inductive_states = values
                if values[sensor - 1] == 1:
                    self.get_logger().info(f"state is 1")
                    if callable(on_success):
                        on_success()
                    else:
                        self.publish_state(on_success)
                else:
                    self.get_logger().info(f"state is 0")
                    if callable(on_failure):
                        on_failure()
                    else:
                        self.publish_state(on_failure)
            except Exception as e:
                self.get_logger().error(f'Failed to parse IND_VALUES: {e}')
                if callable(on_failure):
                    on_failure()
                else:
                    self.publish_state(on_failure)

        self._send_arduino_command('IND_VALUES', '0', 'A',
                                   on_success=_check_sensor,
                                   on_failure=on_failure) 


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
