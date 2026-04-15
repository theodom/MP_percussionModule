[back](../percussionModule.md#motion)
# Motion package

## Components

- [`percussion_motion_node`](./percussion_motion_node.md)
- [`rtde_motions`](./rtde_motions.md)


## Goals

Action takes a [`ExecuteMotion.goal`](../interfaces/action/ExecuteMotion.md) as input parameters. Depending on the `goal.motion_type` it orchestrates the correct robot movement from [`rtde_motions`](./rtde_motions.md). Optional parameters for specific motion types will be added later. 
It returns a `ExecuteMotion.Result` object to the action client ([`task_manager_node`](../Task_Manager/task_manager_node.md))s

### [percussion_motion_node](percussion_motion_node.md)

ROS 2 action server node. Exposes the [`/execute_motion`](../interfaces/action/ExecuteMotion.md) action and dispatches each goal to the correct motion primitive in [`rtde_motions`](./rtde_motions.md). Connects to the UR10e robot via RTDE at startup (blocking). Stateless between goals — no trajectory or history is retained.

**Parameters**:

- `robot_ip`: IP address of the UR10e over RTDE. Default: `169.254.0.22`
- `home_pose`: 6D Cartesian home pose `[x, y, z, rx, ry, rz]` in base frame.
- `default_velocity`: Cartesian velocity for all moves (m/s). Default: `0.2`
- `default_accel`: Cartesian acceleration (m/s²). Default: `0.2`
- `contact_force`: Force threshold for contact detection (N). Default: `1.0`
- `contact_timeout`: Timeout for contact search (s). Default: `5.0`

**Actions**:

- `/execute_motion` [(`ExecuteMotion`)](../interfaces/action/ExecuteMotion.md): Receives a motion goal, executes it, and returns success/failure with final TCP pose.

### [rtde_motions](./rtde_motions.md)

Pure-Python module (no ROS). Contains all RTDE motion primitives. Each function receives an already-connected `RTDEControlInterface` / `RTDEReceiveInterface` pair and returns a `MoveResult`.

**Motion primitives**:

| Function | Behaviour |
| ---      | ---       |
| `move_to_pose` | Move TCP to an absolute Cartesian pose via `moveJ_IK` |
| `move_until_contact` | Move along a direction in base frame until force contact or timeout |
| `move_relative_world` | Apply a 6DOF TCP-frame offset to the current pose via `poseTrans` |
| `compute_face_marker_rvec` | Compute TCP orientation so the tool faces a marker surface, with upright constraint |

> name `move_relative_world` is ambiguous and should be changed.