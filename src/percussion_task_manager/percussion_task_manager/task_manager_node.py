from enum import Enum
from typing import Optional, List
import json

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from std_msgs.msg import String
from percussion_interfaces.srv import TriggerCapture, StartTask
from percussion_interfaces.action import ExecuteMotion, ArduinoCommand
from percussion_interfaces.msg import Pose6D
from .task_modes import MODES


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
        self._state_pub     = self.create_publisher(String, 'state', 10)
        self._state_sub     = self.create_subscription(String, 'state', self._on_state_changed, 10)
        self._capture_client: Client = self.create_client(TriggerCapture, '/percussion/perception/trigger_capture')
        self._motion_client = ActionClient(self, ExecuteMotion, '/percussion/motion/execute_motion')
        self._arduino_client = ActionClient(self, ArduinoCommand, '/percussion/arduino_bridge/arduino_command')

        self._selected_marker: Optional[Pose6D] = None
        self._current_state = TaskState.IDLE
        self._pending_capture_call = None
        self._on_sequence_done = None
        self._sequence: List[dict] = []
        self._mode = None
        self._pause_handler = None

        self.publish_state(self._current_state)
        self.get_logger().info('Task manager node started.')


    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def _on_state_changed(self, _msg: String) -> None:
        state = _msg.data

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

                #--- Add a fixed marker position for at home testing 
                faked_list = [0.00630962,  0.06311957,  0.40705869, -1.72622068,  2.50957927,  0.20299939]
                self._selected_marker = Pose6D()
                self._selected_marker.x  = faked_list[0]
                self._selected_marker.y  = faked_list[1]
                self._selected_marker.z  = faked_list[2]
                self._selected_marker.rx = faked_list[3]
                self._selected_marker.ry = faked_list[4]
                self._selected_marker.rz = faked_list[5]
                self.get_logger().info(f"marker: {self._selected_marker}")
                # #----------------------------------------------------
                if self._selected_marker is None:
                    self.get_logger().error('POSE_ACQUIRED but no marker available')
                    self.publish_state(TaskState.ERROR)
                    return
                if self._mode is None:
                    self.get_logger().error('POSE_ACQUIRED but no task mode selected')
                    self.publish_state(TaskState.ERROR)
                    return
                marker_list = [self._selected_marker.x, self._selected_marker.y, self._selected_marker.z, self._selected_marker.rx, self._selected_marker.ry, self._selected_marker.rz]
                self._pause_handler = lambda step,  resume: self._read_inductive(2, resume, TaskState.ERROR)
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
            self.get_logger().error('Motion action server not available.')
            self.publish_state(TaskState.ERROR)
            return

        goal = ExecuteMotion.Goal()
        goal.motion_type     = step['motion_type']
        goal.step_data       = json.dumps(step)

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
                    on_success(result.message)
                else:
                    self.publish_state(on_success)
            else:
                self.get_logger().error(f'{msg_type} failed: {result.message}')
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
