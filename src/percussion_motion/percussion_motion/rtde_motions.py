"""
rtde_motions.py
---------------
Reusable RTDE motion primitives for the percussion robot.

Each function is a pure motion primitive: it receives an already-initialised
RTDEControlInterface / RTDEReceiveInterface pair plus motion-specific
parameters, executes the move, and returns a MoveResult.

Keeping these separate from the ROS node makes them easy to unit-test,
reuse in scripts, or swap for a MoveIt back-end later.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np
import cv2
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


# ---------------------------------------------------------------------------
# Result type
# ---------------------------------------------------------------------------

class MoveStatus(str, Enum):
    SUCCESS = 'SUCCESS'
    FAILED  = 'FAILED'
    ABORTED = 'ABORTED'


@dataclass
class MoveResult:
    status:  MoveStatus
    message: str = ''
    final_tcp_pose: Optional[List[float]] = field(default=None)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _current_tcp(rtde_r: RTDEReceive) -> List[float]:
    return list(rtde_r.getActualTCPPose())


def apply_offset(
    rtde_c: RTDEControl,
    current_tcp: List[float],
    marker_in_tcp: List[float],
) -> List[float]:
    """
    Convert a marker position (in TCP frame) to base frame using poseTrans.
    Mirrors alignToAruco.py:
        delta_pose = [marker_tcp[0], marker_tcp[1], marker_tcp[2], 0, 0, 0]
        marker_base = rtde_c.poseTrans(current_pose, delta_pose)
    """
    delta = [marker_in_tcp[0], marker_in_tcp[1], marker_in_tcp[2], 0.0, 0.0, 0.0]
    return list(rtde_c.poseTrans(current_tcp, delta))


# ---------------------------------------------------------------------------
# Motion primitives
# ---------------------------------------------------------------------------

def compute_face_marker_rvec(
    marker_rvec_base: List[float],
    world_up_in_base: Optional[List[float]] = None,
) -> List[float]:
    """
    Given the marker's Rodrigues vector in the robot base frame, return the desired
    TCP Rodrigues vector [rx, ry, rz] so that the TCP Z-axis faces the marker surface
    while the tool stays upright (no roll drift around the approach axis).

    Parameters
    ----------
    marker_rvec_base  : [rx, ry, rz]  Rodrigues vector of the marker in base frame
    world_up_in_base  : world "up" direction expressed in base frame coordinates.
                        Defaults to [0, 0, 1] (base Z). Pass a different vector when
                        the robot base is mounted at an angle relative to the world
                        (e.g. 45° roll → [0, sin(45°), cos(45°)]).
    """
    R_marker, _ = cv2.Rodrigues(np.array(marker_rvec_base, dtype=np.float64))
    marker_z = R_marker[:, 2]                          # outward normal of marker face

    z_tcp = -marker_z / np.linalg.norm(marker_z)      # TCP Z points INTO the marker

    world_up = np.array(world_up_in_base, dtype=np.float64) if world_up_in_base is not None \
        else np.array([0.0, 0.0, 1.0])
    world_up /= np.linalg.norm(world_up)

    if abs(np.dot(z_tcp, world_up)) > 0.99:            # degenerate: marker faces along world_up
        world_up = np.array([1.0, 0.0, 0.0])

    x_tcp = np.cross(world_up, z_tcp)
    x_tcp /= np.linalg.norm(x_tcp)
    y_tcp = np.cross(z_tcp, x_tcp)
    y_tcp /= np.linalg.norm(y_tcp)

    R_desired = np.column_stack([x_tcp, y_tcp, z_tcp])
    rvec, _ = cv2.Rodrigues(R_desired)
    return rvec.flatten().tolist()


def move_to_pose(
    rtde_c: RTDEControl,
    rtde_r: RTDEReceive,
    target_pose: List[float],
    velocity: float = 0.2,
    acceleration: float = 0.2,
    blend: float = 0.0,
) -> MoveResult:
    """
    Move the TCP to *target_pose* in Cartesian space (moveJ_IK).

    Parameters
    ----------
    target_pose : [x, y, z, rx, ry, rz]  in the robot base frame
    """

    current_Q = rtde_r.getActualQ()
    try:
        target_Q = rtde_c.getInverseKinematics(target_pose, current_Q)
    except Exception as e:
        return MoveResult(MoveStatus.FAILED, f'IK failed: {e}')
    except len(target_Q) != 6:
        return MoveResult(MoveStatus.FAILED, f'IK failed: no Q vector returned')
    try:
        rtde_c.moveJ(target_Q, velocity, acceleration, blend)
    except Exception as e:
        return MoveResult(MoveStatus.FAILED, f'moveJ failed: {e}')
    # rtde_c.moveJ_IK(target_pose, velocity, acceleration, blend)
    final = _current_tcp(rtde_r)
    return MoveResult(MoveStatus.SUCCESS, 'Reached target pose.', final)



def move_until_contact(
    rtde_c: RTDEControl,
    rtde_r: RTDEReceive,
    direction: List[float],
    tool_speed: float = 0.01,
    force_threshold: float = 10.0,
    timeout_sec: float = 5.0,
) -> MoveResult:
    """
    Move along *direction* in the base frame until contact or timeout.

    Uses forceMode / moveUntilContact if supported by the RTDE version,
    otherwise falls back to a polling loop with moveL + stopL.

    Parameters
    ----------
    direction       : unit vector [dx, dy, dz, 0, 0, 0] in base frame
    tool_speed      : Cartesian speed (m/s) during the search
    force_threshold : contact force in N that triggers a stop (polling mode)
    timeout_sec     : hard timeout; returns ABORTED if contact not found
    """
    try:
        # Preferred: built-in contact detection (UR firmware ≥ 5.9)
        
        success = rtde_c.moveUntilContact(direction)
        final = _current_tcp(rtde_r)
        if success:
            return MoveResult(MoveStatus.SUCCESS, 'Contact detected.', final)
        else:
            return MoveResult(MoveStatus.ABORTED, 'moveUntilContact returned False.', final)

    except AttributeError:
        return MoveResult(MoveStatus.ABORTED, 'Contact timeout.', _current_tcp(rtde_r))

    except Exception as exc:
        return MoveResult(MoveStatus.FAILED, f'move_until_contact failed: {exc}')

def move_relative_world(
    rtde_c: RTDEControl,
    rtde_r: RTDEReceive,
    relative_pose: list[float],
) -> MoveResult:
    """
    Apply a full 6DOF offset (position + rotation) relative to the current TCP pose.
    The offset is expressed in the TCP frame, composed via poseTrans so rotation is applied.
    """
    start = _current_tcp(rtde_r)
    target_pose = list(rtde_c.poseTrans(start, relative_pose))
    return move_to_pose(rtde_c, rtde_r, target_pose)