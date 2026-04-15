# Task_manager_node

## Overview

## Components

- [`__init__`](#init): Node startup — declares parameters, creates services, clients, and publishers.
- [`_on_state_changed`](#_on_state_changed): Central state machine dispatcher; triggers follow-up actions on every state transition.
- [`_build_sequence`](#_build_sequence): Returns the ordered list of motion steps for one percussion task.
- [`_build_return_sequence`](#_build_return_sequence): Returns the ordered list of retract/home steps executed after task completion.
- [`publish_state`](#publish_state): Publishes a new `TaskState` and updates internal state.
- [`start_task_callback`](#start_task_callback): `/start_task` service handler; publishes `TASK_REQUESTED`.
- [`run_capture_service`](#run_capture_service): Sends async `TriggerCapture` request and transitions to `CAPTURING`.
- [`_on_capture_done`](#_on_capture_done): Callback for capture result; selects marker and transitions to `POSE_ACQUIRED` or error.
- [`_execute_next_step`](#_execute_next_step): Pops and dispatches the next step from the active sequence; calls `_on_sequence_done` when empty.
- [`_send_motion_goal`](#_send_motion_goal): Constructs and sends an `ExecuteMotion` action goal for one sequence step.
- [`_on_motion_result`](#_on_motion_result): Handles motion step result; continues sequence or transitions to `ERROR`.
- [`_send_arduino_command`](#_send_arduino_command): Sends a generic `ArduinoCommand` action goal with configurable success/failure transitions.

---

### `init`:

**Declare Parameters**: 

- `capture_service_name`: Name of the service which handles camera image capture
- `target_frame`: unused

**Setup services/topics**:
- `/start_task`: Service which triggers the system start. Callback publishes `TASK_REQUESTED`.
- `/task_manager/state`: Publisher for the current task state. IDLE/CAPTURING/...
- `/task_manager/state` *(subscriber)*: `_on_state_changed` — drives the state machine on every state change.
- `_capture_client`: Client for the `TriggerCapture` service.
- `_motion_client`: `ActionClient` for the `ExecuteMotion` action.
- `_arduino_client`: `ActionClient` for the `ArduinoCommand` action.

**Instance state**:
- `_selected_marker`: `Optional[Pose6D]` — pose of the selected marker from last capture.
- `_sequence`: `List[dict]` — remaining steps in the active motion sequence.
- `_on_sequence_done`: `Optional[callable]` — called when `_sequence` empties; drives post-sequence state transitions.

---

### `_on_state_changed`

**Parameters**:
- `_msg`: `String` — state value published on `/task_manager/state`.

**Return**: /

Central state machine dispatcher. Runs on every state transition and triggers the appropriate follow-up action:

```
TASK_REQUESTED  -->  run_capture_service()
      |
      v
  CAPTURING      -->  (waiting for capture result)
      |
      v
POSE_ACQUIRED   -->  _build_sequence()  +  _execute_next_step()
      |
      v
  AT_MARKER      -->  publish(HAMMERING)  +  _send_arduino_command('HAMMER_REQ', 5)
      |                                             
      | on_failure -> ERROR
      v                                              
                 | 
      on_success v
  HAMMERING      -->  (waiting for Arduino result)
      |
      v
    DONE         -->  publish(RETURNING)
      |
      v
  RETURNING      -->  _build_return_sequence()  +  _execute_next_step()
      |
      v
    IDLE
```

---

### `_build_sequence`: 
**Parameters**: 
 - `marker_pose`: `Pose6D` — marker coordinates in base_link frame.

**Return**: 
- List of dicts containing motion type and movement instructions. 


| Key | Value | info |
| --- | ---   | ---  |
|`motion_type`| `MOVE_TO_MARKER` / `RELATIVE_MOVE` / `MOVE_TO_CONTACT` | type of motion corresponding to available types in percussion_motion package|
| `marker_pose` | `percussion_interfaces/msg/Pose6D` | 6D TCP pose. Marker position in gripper frame for `MOVE_TO_MARKER` type. Otherwise 6 × 0. |
| `approach_offset` | `List[float]` (6 elements) | Offset for `MOVE_TO_MARKER`; direction for `MOVE_TO_CONTACT`; relative TCP delta for `RELATIVE_MOVE`. |
| `contact_force` | `float` *(optional)* | Force threshold (N) for contact moves. Defaults to `2.0` N if omitted. |


> **Notice**: Entire build sequence concept/logic should be reworked to be more generalised. Including dictionary naming and content. 

### `_build_return_sequence`

**Parameters**: /

**Return**:
- List of dicts containing motion type and movement instructions. 

> Logic is equal to `_build_sequence`, but containing different dictionaries/motions (retract + `RETURN_HOME`). To be reworked and generalised along with said function.

---

### `publish_state`:

**Parameters**:

- `state`: `TaskState` enum value.

**Return**: /

Publishes `state.value` on `/task_manager/state` and updates `_current_state`. The subscriber `_on_state_changed` picks it up and drives the next action.

| Value | info |
| ---   | ---  |
| `IDLE` | Default state at startup. Ready to accept `/start_task` service call. |
| `TASK_REQUESTED` | Service call received, capture service will be triggered. |
| `CAPTURING` | Perception service in progress (waiting for vision capture). |
| `POSE_ACQUIRED` | Marker detected. Main sequence building in progress. |
| `MOVING_TO_WEDGELOCK` | *(Deprecated state name, defined but not actively used).* |
| `AT_MARKER` | Main motion sequence complete. Arduino hammer command sent. |
| `HAMMERING` | Percussion event triggered. Waiting for Arduino result. |
| `DONE` | Percussion task completed. Return sequence will be triggered. |
| `RETURNING` | Executing return sequence back to home position. |
| `ERROR` | Failure detected (capture timeout, motion failure, RTDE disconnect, etc.). See log for details. |

---

### `start_task_callback`

**Parameters**:

- `request`: `Trigger.Request`
- `response`: `Trigger.Response`

**Return**: `Trigger.Response`

Publishes `TASK_REQUESTED` (unconditionally — does not check current state). `_on_state_changed` then calls `run_capture_service`.

---

### `run_capture_service`

**Parameters**: /

**Return**: /

Resets `_selected_marker` to `None`, sends an async `TriggerCapture` request, and registers `_on_capture_done` as the callback. Publishes `CAPTURING`.

---

### `_on_capture_done`

**Parameters**:

- `future`: future object returned by the async capture service call.

**Return**: /

Callback triggered when the capture service completes. Handles the following outcomes:

| Outcome | State transition | info |
| ---     | ---              | ---  |
| Exception / no result | `ERROR` | Service call itself failed. |
| `result.success == False` | `ERROR` | Perception node returned failure. |
| No markers detected | `IDLE` | Capture succeeded but scene had no markers. |
| Markers detected | `POSE_ACQUIRED` | Logs all detections, selects `detections[0]`, stores pose in `_selected_marker`. |

> Possible improvement: if no markers detected, request robot move/rotation and retry instead of returning to IDLE.

> Always picks `detections[0]`. No ranking or filtering logic exists.

---

### `_execute_next_step`

**Parameters**: /

**Return**: `bool` — `True` if sequence was empty, `False` otherwise.

Pops the first step from `_sequence` and dispatches it to `_send_motion_goal`.

If the sequence is empty, calls `_on_sequence_done()` (if set) to drive the next state transition. The callback is set by `_on_state_changed` before the sequence starts:
- Main sequence → `_on_sequence_done` publishes `AT_MARKER`
- Return sequence → `_on_sequence_done` publishes `IDLE`

---

### `_send_motion_goal`

**Parameters**:

- `step`: dict with keys `motion_type`, `marker_pose`, `approach_offset`, and optional `contact_force`.

**Return**: /

Constructs an `ExecuteMotion.Goal` from the step dict (using `contact_force` defaulting to `2.0` N) and sends it asynchronously to `/execute_motion`. Goal response and result are handled by inlined closures; `_on_motion_result` is called on completion.

If `marker_pose` is `None` or the action server is not ready, immediately transitions to `ERROR`.

---

### `_on_motion_result`

**Parameters**:

- `future`: future returned by `get_result_async`.

**Return**: /

Handles the result of a completed motion step.

| Outcome | Behaviour |
| ---     | ---       |
| `result.success == True` | Calls `_execute_next_step` to continue the sequence. |
| `result.success == False` | Logs error, clears sequence, clears `_on_sequence_done`, transitions to `ERROR`. |
| Exception (server crash) | Logs error, clears sequence, clears `_on_sequence_done`, transitions to `ERROR`. |

---

### `_send_arduino_command`

**Parameters**:

- `msg_type`: `str` — command type (e.g. `'HAMMER_REQ'`)
- `data`: `str` — command payload
- `msg_info`: `str` — optional metadata
- `on_success`: `TaskState` or `callable` — state to publish or function to call on success
- `on_failure`: `TaskState` or `callable` — state to publish or function to call on failure

**Return**: /

Sends a generic `ArduinoCommand` action goal to `/arduino_command`. On result, calls or publishes `on_success` / `on_failure` depending on `result.success`. If the action server is not ready, transitions to `ERROR` immediately.
