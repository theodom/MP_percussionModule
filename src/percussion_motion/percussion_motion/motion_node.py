"""
motion_node.py
--------------
ROS 2 node that receives a marker pose from the task manager and orchestrates
robot motion via the RTDE back-end (rtde_motions.py).

Architecture
------------
  TaskManagerNode  --[ExecuteMotion action]--> MotionNode
                                                   |
                                              rtde_motions.py
                                                   |
                                              UR robot (RTDE)

The node exposes one ROS 2 action server:

  /execute_motion  (percussion_interfaces/action/ExecuteMotion)

The action goal carries:
  - motion_type      : string  (see MotionType enum)
  - marker_pose      : percussion_interfaces/Pose6D  (in robot base frame)
  - pose_offset  : float64[6]  pose_offset in TCP frame applied before the final move

Extending for new motion types
-------------------------------
1. Add a new entry to MotionType.
2. Add a branch in _execute_motion_sequence().
3. Implement the primitive in rtde_motions.py if needed.
No changes to the interface definition are required.
"""

from __future__ import annotations

from enum import Enum
from typing import List
import json

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from percussion_interfaces.action import ExecuteMotion
from percussion_interfaces.msg import Pose6D

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from percussion_motion import rtde_motions as motions
from percussion_motion.rtde_motions import (
    MoveStatus, MoveResult, move_to_pose, apply_offset as apply_pose_offset,
    compute_face_marker_rvec, compute_snap_to_principal_rvec
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _list_to_pose6d(lst: List[float]) -> Pose6D:
    p = Pose6D()
    p.x, p.y, p.z = lst[0], lst[1], lst[2]
    p.rx, p.ry, p.rz = lst[3], lst[4], lst[5]
    return p



# ---------------------------------------------------------------------------
# Motion type registry
# ---------------------------------------------------------------------------

class MotionType(str, Enum):
    MOVE_TO_MARKER  = 'MOVE_TO_MARKER'   # approach via pose_offset, then move to marker
    MOVE_TO_CONTACT = 'MOVE_TO_CONTACT'  # approach via pose_offset, then search for contact
    RETURN_HOME     = 'RETURN_HOME'      # go back to home joint configuration
    RELATIVE_MOVE   = 'RELATIVE_MOVE'    # apply pose_offset to current TCP pose
    JOINT_MOVE      = 'JOINT_MOVE'
    MOVE_TO_FORCE   = 'MOVE_TO_FORCE'


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MotionNode(Node):

    def __init__(self) -> None:
        super().__init__('motion')

        # --- Parameters ---
        self.declare_parameter('robot_ip',         '169.254.0.22')
        self.declare_parameter('home_pose', [0.0652, 0.4498, 0.4911, 1.9767, 0.9942, 1.6870]
)
        self.declare_parameter('default_velocity', 0.6)
        self.declare_parameter('default_accel',    0.3)
        self.declare_parameter('contact_force',    1.0)
        self.declare_parameter('contact_timeout',  5.0)

        robot_ip          = self.get_parameter('robot_ip').get_parameter_value().string_value
        self._home_pose   = list(self.get_parameter('home_pose').get_parameter_value().double_array_value)
        self._def_vel     = self.get_parameter('default_velocity').get_parameter_value().double_value
        self._def_acc     = self.get_parameter('default_accel').get_parameter_value().double_value
        self._contact_frc = self.get_parameter('contact_force').get_parameter_value().double_value
        self._contact_tmt = self.get_parameter('contact_timeout').get_parameter_value().double_value

        # --- State publisher ---
        self._state_pub = self.create_publisher(String, 'state', 10)

        # --- Action server (register before blocking RTDE connect) ---
        self._rtde_ready = False
        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ExecuteMotion,
            'execute_motion',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=cb_group,
        )

        # --- RTDE connection (blocking) ---
        self._publish_state('CONNECTING')
        self.get_logger().info(f'Connecting to robot at {robot_ip} …')
        try:
            self._rtde_c = RTDEControl(robot_ip)
            self._rtde_r = RTDEReceive(robot_ip)
        except Exception as e:
            self._rtde_ready = False
            self.get_logger().info(f'Failed to connect to robot: {e}')
            self._state_pub("ERROR")
        self._rtde_ready = True
        self.get_logger().info('RTDE connection established.')

        self._publish_state('IDLE')
        self.get_logger().info('Percussion motion node ready.')

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not self._rtde_ready:
            self.get_logger().warn('Robot not connected yet, rejecting goal.')
            return GoalResponse.REJECT
        motion_type = goal_request.motion_type
        if motion_type not in [m.value for m in MotionType]:
            self.get_logger().warn(f'Unknown motion type requested: {motion_type}')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal: {motion_type}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received — stopping robot.')
        try:
            self._rtde_c.stopJ(2.0)
        except Exception:
            pass
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        goal        = goal_handle.request
        motion_type = goal.motion_type

        try:
            step = json.loads(goal.step_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse step_data JSON: {e}')
            result = ExecuteMotion.Result()
            result.success = False
            result.message = f'JSON parse error: {e}'
            goal_handle.abort()
            return result

        self.get_logger().info(f'Executing motion: {motion_type}')
        self._publish_state(motion_type)

        result = ExecuteMotion.Result()

        try:
            move_result = self._execute_motion_sequence(goal_handle, motion_type, step)
            result.success        = move_result.status == MoveStatus.SUCCESS
            result.message        = move_result.message
            result.final_tcp_pose = (
                _list_to_pose6d(move_result.final_tcp_pose)
                if move_result.final_tcp_pose else Pose6D()
            )

        except Exception as exc:
            self.get_logger().error(f'Unhandled exception in motion execution: {exc}')
            result.success = False
            result.message = str(exc)

        if result.success:
            self._publish_state('IDLE')
            goal_handle.succeed()
        else:
            self._publish_state('ERROR')
            goal_handle.abort()

        return result

    # ------------------------------------------------------------------
    # Motion orchestration
    # ------------------------------------------------------------------

    def _execute_motion_sequence(
        self,
        goal_handle: ServerGoalHandle,
        motion_type: str,
        step: dict,
    ) -> MoveResult:

        feedback = ExecuteMotion.Feedback()

        def send_feedback(phase: str, text: str = None) -> None:
            feedback.current_phase = phase
            goal_handle.publish_feedback(feedback)
            if text is None:
                self.get_logger().info(f'  [{motion_type}] phase: {phase}')
            else:
                self.get_logger().info(f'  [{motion_type}] phase: {phase}')
                self.get_logger().info(f'Extra: {text}')

        # ---- MOVE_TO_MARKER ----------------------------------------
        if motion_type == MotionType.MOVE_TO_MARKER:
            marker_pose = list(step.get("marker_pose"))
            pose_offset = list(step.get("approach_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            invert_tcp = bool(step.get("invert_tcp", False))

            current_tcp = list(self._rtde_r.getActualTCPPose())

            # Full 6DOF: get marker pose (position + orientation) in base frame
            marker_base = np.array(self._rtde_c.poseTrans(current_tcp, marker_pose))

            # World Z expressed in base_link frame, accounting for the 45° roll mount
            # (world → base_link: roll=0.785398 rad around X)
            # R_x(-45°)^T * [0,0,1] = [0, sin(-45°), cos(-45°)]
            _s45 = np.sin(np.deg2rad(45.0))
            _c45 = np.cos(np.deg2rad(45.0))
            _s = np.sin(np.deg2rad(-45.0))
            _c = np.cos(np.deg2rad(-45.0))
            world_up_in_base = [0.0, _s, _c]
            if invert_tcp:
                world_up_in_base = [-v for v in world_up_in_base]

            # 4 world horizontal principal directions expressed in base frame
            principal_dirs_base = [
                [1.0, 0.0, 0.0],        # +X_world
                [-1.0, 0.0, 0.0],       # -X_world
                [0.0, _c45, _s45],      # +Y_world
                [0.0, -_c45, -_s45],    # -Y_world
            ]

            # Snap TCP Z to closest world principal direction
            rvec_desired = compute_snap_to_principal_rvec(
                marker_base[3:], principal_dirs_base, world_up_in_base
            )
            marker_base[3:] = rvec_desired

            # Apply standoff (-10 cm along base X)
            marker_base[:3] -= pose_offset[:3]

            #marker_base[5] = 0 if -0.30 < current_tcp[5] < 0.30 else 3.14

            send_feedback('MOVING_TO_MARKER')
            result = move_to_pose(self._rtde_c, self._rtde_r, marker_base,
                                  velocity=self._def_vel, acceleration=self._def_acc)
            if result.status == MoveStatus.SUCCESS:
                send_feedback('MARKER_REACHED')
            return result

        # ---- MOVE_TO_CONTACT ----------------------------------------
        elif motion_type == MotionType.MOVE_TO_CONTACT:
            direction = list(step.get("direction"))

            self.get_logger().info(f'Moving to contact in direction: {direction}')

            send_feedback('SEARCHING_CONTACT')
            result = motions.move_until_contact(
                self._rtde_c, self._rtde_r,
                direction=direction,
                timeout_sec=self._contact_tmt,
            )
            if result.status == MoveStatus.SUCCESS:
                send_feedback('CONTACT_MADE')
            return result

        # ---- RETURN_HOME --------------------------------------------
        elif motion_type == MotionType.RETURN_HOME:
            send_feedback('RETURNING_HOME')
            home_pose = list(step.get("home_pose"))
            home = home_pose if home_pose is not None else self._home_pose
            result = move_to_pose(self._rtde_c, self._rtde_r, home,
                                  velocity=self._def_vel, acceleration=self._def_acc)
            if result.status == MoveStatus.SUCCESS:
                send_feedback('HOME_REACHED')
            return result

        # ---- RELATIVE_MOVE ------------------------------------------
        elif motion_type == MotionType.RELATIVE_MOVE:
            relative_goal = list(step.get("relative_pose"))
            Q_near_raw = step.get('Q_near')
            Q_near = list(Q_near_raw) if Q_near_raw is not None else None

            send_feedback('MOVING')
            return motions.move_relative_tcp(self._rtde_c, self._rtde_r, relative_goal, Q_near, self._def_vel, self._def_acc)

        # ---- JOINT MOVE ---------------------------------------------
        elif motion_type == MotionType.JOINT_MOVE:
            joint_goal = list(step.get('goal_Q'))
            send_feedback("MOVING")
            return motions.move_joints(self._rtde_c, self._rtde_r, joint_goal, self._def_vel, self._def_acc)

        # ---- MOVE_TO_FORCE -------------------------------------------
        elif motion_type == MotionType.MOVE_TO_FORCE:
            direction = list(step.get('direction'))
            force = float(step.get('force_threshold'))
            send_feedback("MOVING")
            return motions.move_until_force(self._rtde_c, self._rtde_r, direction, force)
        

        else:
            return MoveResult(MoveStatus.FAILED, f'Unhandled motion type: {motion_type}')
        


    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)




# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — stopping robot script.')
        try:
            node._rtde_c.stopScript()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
