[back](./motion.md)
# rtde_motions

## Overview

Pure-Python module. No ROS dependency. Contains all RTDE motion primitives used by [`percussion_motion_node`](./percussion_motion_node.md). Each function receives an already-connected `RTDEControlInterface` / `RTDEReceiveInterface` pair, executes a move, and returns a `MoveResult`.


## Components

- [`MoveStatus` / `MoveResult`](#moveresult): return types used by all primitives
- `_current_ctp`: Gets TCP pose as 6D List from `rtde_r`
- [`apply_offset`](#apply_offset): convert a TCP-frame delta to base frame via `poseTrans`
- [`compute_face_marker_rvec`](#compute_face_marker_rvec): compute desired TCP orientation to face a marker
- [`compute_snap_to_principal_rvec`](#compute_snap_to_principal_rvec): Aligns TCP to one of four principal directions
- [`move_to_pose`](#move_to_pose): move to a Cartesian pose via `moveJ_IK`
- [`move_until_contact`](#move_until_contact): move in a direction until force contact or timeout
- [`move_relative_world`](#move_relative_world): apply a 6DOF TCP-frame offset to the current pose

---

### `apply_offset`

**Parameters**:

- `rtde_c`: `RTDEControlInterface`
- `current_tcp`: `List[float]` — current TCP pose in base frame
- `marker_in_tcp`: `List[float]` — marker position expressed in TCP frame

**Return**:

- `List[float]` — marker position in base frame

Converts a TCP-frame position delta to base frame using `rtde_c.poseTrans`. Only the translational part `[x, y, z]` is used; orientation is zeroed out in the delta.

---

### `MoveResult`

Dataclass returned by all motion primitives.

| Field | Type | Info |
| ---   | ---  | ---  |
| `status` | `MoveStatus` | `SUCCESS`, `FAILED`, or `ABORTED` |
| `message` | str | Human-readable description of outcome |
| `final_tcp_pose` | `List[float]` or `None` | TCP pose `[x,y,z,rx,ry,rz]` after the move, if available |

---


### `compute_face_marker_rvec`

**Parameters**:

- `marker_rvec_base`: `List[float]` — Rodrigues vector of the marker in base frame
- `world_up_in_base`: `List[float]` or `None` — world "up" direction in base frame coordinates. Defaults to `[0, 0, 1]`.

**Return**:

- `List[float]` — desired TCP Rodrigues vector `[rx, ry, rz]`

Computes the TCP orientation so that the TCP Z-axis points into the marker surface, while keeping the tool upright (no roll drift). Used in `MOVE_TO_MARKER` to align the tool face-to-face with the marker.

The `world_up_in_base` argument must account for the physical robot mount angle. For the current 45° X-axis mount:

```
world_up_in_base = [0, sin(-45°), cos(-45°)]
```

> If the marker normal is nearly parallel to `world_up_in_base` (dot product > 0.99), falls back to `[1, 0, 0]` to avoid a degenerate cross product.

---

### `compute_snap_to_principal_rvec`

**Parameters**:

- `marker_rvec_base`: `List[float]` — Rodrigues vector `[rx, ry, rz]` of the marker in the robot base frame
- `principal_dirs_base`: `List[List[float]]` — candidate TCP Z-axis unit vectors in the base frame; the function snaps to the closest one
- `world_up_in_base`: `List[float]` or `None` — world "up" direction in base frame. Defaults to `[0, 0, 1]`

**Return**:

- `List[float]` — desired TCP Rodrigues vector `[rx, ry, rz]`

Variant of `compute_face_marker_rvec` that constrains the TCP approach to a finite set of pre-defined directions instead of tracking the exact marker normal.

Converts `marker_rvec_base` to a rotation matrix, extracts the marker's outward Z-axis, negates it to get the ideal TCP Z direction, then picks the entry from `principal_dirs_base` with the highest dot product against that ideal direction. An orthonormal TCP frame (X, Y, Z) is built using `world_up_in_base` as the reference up-vector and converted back to a Rodrigues vector.

> If the chosen principal direction is nearly parallel to `world_up_in_base` (dot product > 0.99), the up fallback switches to `[1, 0, 0]` to avoid a degenerate cross product.

---

### `move_to_pose`

**Parameters**:

- `rtde_c`: `RTDEControlInterface`
- `rtde_r`: `RTDEReceiveInterface`
- `target_pose`: `List[float]` — `[x, y, z, rx, ry, rz]` in robot base frame
- `velocity`: float — Cartesian velocity (m/s). Default: `0.2`
- `acceleration`: float — Cartesian acceleration (m/s²). Default: `0.2`
- `blend`: float — blend radius. Default: `0.0`

**Return**:

- `MoveResult`

Moves the TCP to `target_pose` using `rtde_c.moveJ_IK` (inverse kinematics joint move to a Cartesian target). Returns final TCP pose on success.

---

### `move_until_contact`

**Parameters**:

- `rtde_c`: `RTDEControlInterface`
- `rtde_r`: `RTDEReceiveInterface`
- `direction`: `List[float]` — 6-element vector `[dx, dy, dz, 0, 0, 0]` in base frame
- `tool_speed`: float — Cartesian speed during search (m/s). Default: `0.01`
- `force_threshold`: float — contact force in N for polling fallback. Default: `10.0`
- `timeout_sec`: float — hard timeout (s). Default: `5.0`

**Return**:

- `MoveResult`

Moves along `direction` until contact is detected or timeout expires.

| Outcome | Status | Info |
| ---     | ---    | ---  |
| `moveUntilContact` returns `True` | `SUCCESS` | Built-in firmware contact detection triggered |
| `moveUntilContact` returns `False` | `ABORTED` | Firmware reported no contact |
| `AttributeError` (firmware < 5.9) | `ABORTED` | `moveUntilContact` not available; fallback not implemented |
| Other exception | `FAILED` | Unexpected RTDE error |

> `tool_speed` parameter is accepted but currently unused — `moveUntilContact` handles speed internally via firmware. They are kept for the polling fallback path which is not yet implemented.

---

### `move_relative_world`

**Parameters**:

- `rtde_c`: `RTDEControlInterface`
- `rtde_r`: `RTDEReceiveInterface`
- `relative_pose`: `List[float]` — 6DOF offset `[x, y, z, rx, ry, rz]` in TCP frame

**Return**:

- `MoveResult`

Reads the current TCP pose, applies `relative_pose` via `rtde_c.poseTrans` (full 6DOF TCP-frame composition, not simple vector addition), and calls `move_to_pose` to the resulting target. Rotation components in `relative_pose` are applied as a TCP-frame rotation.

> name is misleading, shoudl be move_relative_TCP 