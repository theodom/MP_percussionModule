[back](./task_manager.md)
# task_modes

## Overview

Pure-Python module (no ROS node). Defines the **task modes** the [`task_manager_node`](./task_manager_node.md) can run, and the motion sequences for each. A mode bundles three sequence builders (home, main, return) plus the Arduino percussion command. The task manager looks the active mode up in the `MODES` registry by the name passed to [`/start_task`](../interfaces/srv/StartTask.md).

Each sequence is a `List[dict]`; every dict has a `motion_type` and motion-specific keys that are JSON-encoded and sent to the [Motion package](../Motion/motion.md) as `ExecuteMotion.step_data`. A dict of `{'motion_type': 'PAUSE'}` halts the sequence so the task manager can inject a side-effect (e.g. an inductive sensor read) before resuming.

## Components

- [`_make_pose6d`](#_make_pose6d): Helper to build a [`Pose6D`](../interfaces/msg/Pose6D.md).
- [Mode builders](#mode-builders): `_<mode>_home` / `_<mode>_sequence` / `_<mode>_return` for each mode.
- [`MODES`](#modes-registry): Registry mapping mode name → builders + Arduino config.

---

### `_make_pose6d`

**Parameters**: `x, y, z, rx, ry, rz` (all default `0.0`).

**Return**: a `Pose6D` populated from the arguments.

---

### Mode builders

For each mode `<mode>`:

| Function | Parameters | Return |
| ---      | ---        | ---    |
| `_<mode>_home()` | / | Home/preparation sequence (e.g. a `JOINT_MOVE` to a safe observation pose). |
| `_<mode>_sequence(marker_pose)` | `marker_pose`: `List[float]` (6D) | Main sequence built around the detected marker — approach, contact detection, repositioning, and a final `MOVE_TO_FORCE`. |
| `_<mode>_return()` | / | Retract + return-home sequence executed after percussion. |

Step dicts use the `motion_type` keys documented in the [Motion package](../Motion/motion.md), e.g. `MOVE_TO_MARKER` (`marker_pose`, `approach_offset`, `invert_tcp`), `MOVE_TO_CONTACT` (`direction`), `RELATIVE_MOVE` (`relative_pose`, optional `Q_near`), `JOINT_MOVE` (`goal_Q`), `MOVE_TO_FORCE` (`direction`, `force_threshold`), and `PAUSE`.

**Current modes:**

| Mode | Main sequence | Notes |
| ---  | ---           | ---   |
| `FIXING` | Spiral approach + force application for fastener tightening; includes a `PAUSE` (inductive check). | Arduino `HAMMER_REQ`, `data='8'`. |
| `LOOSENING` | Inverted-orientation (`invert_tcp`) approach with distinct contact points for fastener loosening. | Arduino `HAMMER_REQ`, `data='5'`. |

> The numeric offsets/forces in these sequences are hand-tuned to the rig and change frequently; treat them as calibration values, not fixed constants.

---

### `MODES` registry

Maps a mode name to its configuration dict:

```python
MODES = {
    'FIXING': {
        'name': 'FIXING',
        'build_home_sequence':   _fixing_home,
        'build_sequence':        _fixing_sequence,
        'build_return_sequence': _fixing_return,
        'arduino': {'msg_type': 'HAMMER_REQ', 'data': '8', 'msg_info': 'a'},
    },
    'LOOSENING': { ... },
}
```

| Key | info |
| --- | ---  |
| `name` | Mode name, mirrored into `SystemState.task_mode`. |
| `build_home_sequence` | Callable → home sequence (run in `TASK_REQUESTED`). |
| `build_sequence` | Callable taking the marker pose → main sequence (run in `POSE_ACQUIRED`). |
| `build_return_sequence` | Callable → return sequence (run in `RETURNING`). |
| `arduino` | `msg_type` / `data` / `msg_info` for the percussion `ArduinoCommand` (sent in `HAMMERING`). |

**Adding a new mode:** create `_<mode>_home`, `_<mode>_sequence`, `_<mode>_return`, then register them in `MODES` with the Arduino config. The task manager dispatches automatically via the `mode` argument to `/start_task` — no node changes required.
