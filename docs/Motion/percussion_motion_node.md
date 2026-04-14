# percussion_motion_node

## Overview

ROS 2 action server node. Receives [`ExecuteMotion`](../interfaces/action/ExecuteMotion.md) goals from the [`task_manager_node`](../Task_Manager/task_manager_node.md), selects the correct motion primitive based on `motion_type`, and delegates execution to [`rtde_motions`](./rtde_motions.md).

Connects to the UR10e robot via RTDE at startup (blocking). The action server is registered before the RTDE connection so goals can be queued while the robot connects.

## Components

- Node: `PercussionMotionNode`
    - `__init__`
    - `_goal_callback`
    - `_cancel_callback`
    - `_execute_callback`
    - `_execute_motion_sequence`
    - `_publish_state`

---

### `__init__`

**Declare Parameters**:

- `robot_ip`: IP address of the UR10e robot over RTDE. Default: `169.254.0.22`
- `home_pose`: 6D Cartesian home pose `[x, y, z, rx, ry, rz]` in base frame. 
- `default_velocity`: Cartesian velocity for all moves (m/s). Default: `0.2`
- `default_accel`: Cartesian acceleration for all moves (m/s²). Default: `0.2`
- `contact_force`: Force threshold for contact detection (N). Default: `1.0`
- `contact_timeout`: Timeout for contact search (s). Default: `5.0`

**Setup**:

- `/motion_node/state`: Publisher for current motion node state (String)
- `/execute_motion`: Action server (`ExecuteMotion`)
- `RTDEControlInterface` + `RTDEReceiveInterface`: Blocking connection to robot at startup


---

### `_goal_callback`

**Parameters**:

- `goal_request`: incoming `ExecuteMotion.Goal`

**Return**:

- `GoalResponse.ACCEPT` or `GoalResponse.REJECT`

Rejects the goal if RTDE is not yet connected, or if `motion_type` is not a recognised `MotionType` enum value.

---

### `_cancel_callback`

**Parameters**:

- `goal_handle`

**Return**:

- `CancelResponse.ACCEPT`

Accepts all cancel requests. Attempts to stop the robot immediately via `rtde_c.stopJ(2.0)`.

---

### `_execute_callback`

**Parameters**:

- `goal_handle`: `ServerGoalHandle`

**Return**:

- `ExecuteMotion.Result`

Entry point for goal execution. Unpacks the goal, calls `_execute_motion_sequence`, and translates the returned `MoveResult` into an `ExecuteMotion.Result`. Calls `goal_handle.succeed()` or `goal_handle.abort()` accordingly.

Any unhandled exception is caught here and returned as a failed result with the exception message.

---

### `_execute_motion_sequence`

**Parameters**:

- `goal_handle`: `ServerGoalHandle` — used to publish feedback phases
- `motion_type`: str — one of the `MotionType` enum values
- `marker_pose`: `List[float]` — 6D pose `[x, y, z, rx, ry, rz]`
- `pose_offset`: `List[float]` — 6-element offset, meaning depends on motion type

**Return**:

- `MoveResult`

Core dispatch function. Selects and executes the correct motion primitive based on `motion_type`:

| Motion type | Behaviour |
| ---         | ---       |
| `MOVE_TO_MARKER` | Converts `marker_pose` from TCP frame to base frame via `poseTrans`. Computes face-to-marker TCP orientation via `compute_face_marker_rvec` (accounts for 45° robot mount). Subtracts 0.10 m on base-frame X as standoff. Calls `move_to_pose` twice to that position. `pose_offset` is ignored. |
| `MOVE_TO_CONTACT` | Converts `marker_pose` to base frame. Adds `pose_offset[:3]` in base frame as approach position and moves there. Then calls `move_until_contact` with `pose_offset` as the contact direction. Force threshold from `contact_force` parameter (default: 1.0 N). |
| `RETURN_HOME` | Calls `move_to_pose` with `home_pose` parameter. |
| `RELATIVE_MOVE` | Calls `move_relative_world` — applies `pose_offset` as a 6DOF TCP-frame offset via `poseTrans`. |

Publishes feedback phases via `goal_handle.publish_feedback` at each stage of the motion.

> `MOVE_TO_MARKER` moves to the standoff position twice — the second call is redundant. Two-phase approach/strike is not yet implemented.

---

### `_publish_state`

**Parameters**:

- `state`: str — state label to publish

**Return**:

/

Publishes the given string to `/motion_node/state`.
