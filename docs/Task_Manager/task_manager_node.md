[back](./task_manager.md)
# Task_manager_node

## Overview

ROS 2 node `task_manager` — the high-level orchestrator. It runs an **event-driven state machine**: every handler does its work and calls `publish_state(next)`, which publishes a `SystemState` message on the `state` topic; the node's own subscriber `_on_state_changed` then dispatches the follow-up action. Service and action calls are all asynchronous, so the machine advances through callbacks rather than a blocking loop.

The motion sequences themselves live in [`task_modes`](./task_modes.md) (the `MODES` registry); structured failure recovery lives in [`error_handler`](./error_handler.md).

## Components

- [`__init__`](#init): Node startup — declares parameters, creates the service, publisher/subscriber, clients, timer, instance state, and the `ErrorHandler`.
- [`_on_state_changed`](#_on_state_changed): Central state-machine dispatcher; triggers follow-up actions on every *changed* state.
- [`publish_state`](#publish_state): Records the previous state, sets the current state, and publishes a `SystemState`.
- [`_publish_system_state`](#_publish_system_state): Builds and publishes the `SystemState` message (also the 1 Hz heartbeat).
- [`_raise_inductive_error`](#_raise_inductive_error): Pause-handler failure path; tags an inductive error and transitions to `ERROR`.
- [`start_task_callback`](#start_task_callback): `/start_task` service handler; validates the mode and publishes `TASK_REQUESTED`.
- [`run_capture_service`](#run_capture_service): Sends an async `TriggerCapture` request and transitions to `CAPTURING`.
- [`_on_capture_done`](#_on_capture_done): Capture-result callback; selects a marker, retries once on no detections, or errors.
- [`_execute_next_step`](#_execute_next_step): Pops and dispatches the next sequence step (handling `PAUSE`); calls `_on_sequence_done` when empty.
- [`_send_motion_goal`](#_send_motion_goal): Constructs and sends an `ExecuteMotion` action goal for one step (JSON `step_data`).
- [`_on_motion_result`](#_on_motion_result): Handles a motion step result; continues the sequence or transitions to `ERROR`.
- [`_send_arduino_command`](#_send_arduino_command): Sends a generic `ArduinoCommand` action goal with configurable success/failure transitions.
- [`_read_inductive`](#_read_inductive): Reads an inductive sensor via `IND_VALUES` and dispatches on the parsed value.

---

### `init`:

**Declare Parameters**:

- `target_frame`: `string` (default `'base'`) — currently unused.

**Setup services/topics**:
- `/start_task` [(`StartTask`)](../interfaces/srv/StartTask.md): triggers a task. Callback validates the mode and publishes `TASK_REQUESTED`.
- `state` *(publisher)*: current status as [`SystemState`](../interfaces/msg/SystemState.md) (not a plain `String`).
- `state` *(subscriber)*: `_on_state_changed` — drives the state machine on every *changed* state.
- `_capture_client`: client for `/percussion/perception/trigger_capture` (`TriggerCapture`).
- `_motion_client`: `ActionClient` for `/percussion/motion/execute_motion` (`ExecuteMotion`).
- `_arduino_client`: `ActionClient` for `/percussion/arduino_bridge/arduino_command` (`ArduinoCommand`).
- `_timer`: 1 Hz timer calling `_publish_system_state` (heartbeat).

**Instance state**:
- `_selected_marker`: `Optional[Pose6D]` — pose of the selected marker from the last capture.
- `_current_state` / `_previous_state`: current `TaskState`; last non-`ERROR` state (used by the error handler to resume).
- `_last_processed_state`: dedup guard so the 1 Hz heartbeat does not re-trigger state logic.
- `_sequence`: `List[dict]` — remaining steps in the active motion sequence.
- `_on_sequence_done`: `Optional[callable]` — called when `_sequence` empties; drives post-sequence transitions.
- `_mode`: the active entry from the `MODES` registry (set by `/start_task`).
- `_pause_handler`: `Optional[callable]` — invoked by `PAUSE` steps as `handler(step, resume)`.
- `_no_marker_retries`: inline "retry capture once" budget for empty detections.
- `_inductive_states`, `_latest_tcp_pose`, `_error_message`, `_detected_marker_id`: fields mirrored into `SystemState`.
- `_error_handler`: [`ErrorHandler`](./error_handler.md) instance.

---

### `_on_state_changed`

**Parameters**:
- `msg`: [`SystemState`](../interfaces/msg/SystemState.md) — published on the `state` topic. Only `msg.task_state` is read here.

**Return**: /

Central state-machine dispatcher. It first dedupes: if `msg.task_state == _last_processed_state` (e.g. the 1 Hz heartbeat re-publishing the same state) it returns immediately. Otherwise it dispatches:

```
  start_task(mode)
      |
      v
TASK_REQUESTED  -->  build_home_sequence()  +  _execute_next_step()   (on done -> AT_HOME)
      |
      v
   AT_HOME       -->  run_capture_service()
      |
      v
  CAPTURING      -->  (await capture result -> POSE_ACQUIRED | retry | ERROR)
      |
      v
POSE_ACQUIRED   -->  set _pause_handler (read inductive sensor 2)
                     build_sequence(marker)  +  _execute_next_step()   (on done -> AT_MARKER)
      |
      v
  AT_MARKER      -->  publish(HAMMERING)
      |
      v
  HAMMERING      -->  _send_arduino_command(mode['arduino'])   on_success -> DONE, on_failure -> ERROR
      |
      v
    DONE         -->  publish(RETURNING)
      |
      v
  RETURNING      -->  build_return_sequence()  +  _execute_next_step()  (on done -> IDLE)
      |
      v
    IDLE         -->  reset error/inductive/marker/retry fields + error_handler.reset_retries()

  ERROR          -->  error_handler.handle_error(_error_message, context={previous_state, mode})
```

---

### `publish_state`:

**Parameters**:
- `state`: `TaskState` enum value.

**Return**: /

Records `_previous_state` (only while the current state is **not** `ERROR`, so the error handler can resume the in-flight step), sets `_current_state`, calls `_publish_system_state()`, and logs `State -> <value>`.

| Value | info |
| ---   | ---  |
| `IDLE` | Default at startup. Ready to accept `/start_task`. Resets `_error_message`, `_inductive_states`, `_detected_marker_id`, `_no_marker_retries`, and the error handler's retry counters. |
| `TASK_REQUESTED` | Mode selected; about to run the home sequence. |
| `AT_HOME` | Home sequence complete; capture will be triggered. |
| `CAPTURING` | Perception service in progress (awaiting vision capture). |
| `POSE_ACQUIRED` | Marker detected; building the main sequence and arming the inductive `PAUSE` handler. |
| `MOVING_TO_WEDGELOCK` | *(Deprecated state name, defined but not actively used).* |
| `AT_MARKER` | Main motion sequence complete; transitions to `HAMMERING`. |
| `HAMMERING` | Percussion event triggered. Awaiting Arduino result. |
| `DONE` | Percussion completed; return sequence will be triggered. |
| `RETURNING` | Executing the mode return sequence back to home. |
| `ERROR` | Failure detected (capture timeout, motion failure, RTDE disconnect, inductive fault, …). Routed to the [`error_handler`](./error_handler.md). |

---

### `_publish_system_state`

**Parameters**: /

**Return**: /

Builds a [`SystemState`](../interfaces/msg/SystemState.md) from the current fields (`task_state`, `task_mode`, `inductive_states`, `tcp_pose`, `error_message`, `detected_marker_id`) and publishes it on `state`. Called on every `publish_state` and once per second by the heartbeat timer.

---

### `_raise_inductive_error`

**Parameters**: /

**Return**: /

Failure path for the inductive `PAUSE` handler. Sets `_error_message = 'INDUCTIVE sensor 2 not detected'` and publishes `ERROR`, so the error handler classifies it as `INDUCTIVE_FAULT`.

---

### `start_task_callback`

**Parameters**:
- `request`: `StartTask.Request` — `request.mode` (`"FIXING"` / `"LOOSENING"`, case-insensitive).
- `response`: `StartTask.Response`.

**Return**: `StartTask.Response` (`success`, `message`).

Validates `request.mode.upper()` against the `MODES` registry. Unknown modes return `success=False` with the list of valid modes and **no** state change. Valid modes set `_mode`, publish `TASK_REQUESTED`, and return `success=True`.

---

### `run_capture_service`

**Parameters**: /

**Return**: /

Resets `_selected_marker` to `None`, sends an async `TriggerCapture` request, registers `_on_capture_done` as the callback, and publishes `CAPTURING`. Warns (but still sends) if the service is not yet reported ready.

---

### `_on_capture_done`

**Parameters**:
- `future`: future returned by the async capture call.

**Return**: /

| Outcome | State transition | info |
| ---     | ---              | ---  |
| Exception / `None` result | `ERROR` | The service call itself failed. |
| `result.success == False` | `ERROR` | Perception node returned failure. |
| No detections, retry budget left | re-run home sequence → capture again | Re-runs `build_home_sequence()` once (`_no_marker_retries < 1`), then retries capture. |
| No detections, budget exhausted | `ERROR` | `_error_message = 'MARKER NOT FOUND in view'` (classified `MARKER_NOT_FOUND`). |
| Detections present | `POSE_ACQUIRED` | Logs all detections, selects `detections[0]`, stores pose + `marker_id`. |

> Always picks `detections[0]`. No ranking or filtering logic exists.

---

### `_execute_next_step`

**Parameters**: /

**Return**: `bool` — `True` if the sequence was empty, `False` otherwise.

Pops the first step from `_sequence`. If the sequence is empty, calls `_on_sequence_done()` (if set) to drive the next transition. The callback is armed by `_on_state_changed` before each sequence:
- Home sequence → publishes `AT_HOME`
- Main sequence → publishes `AT_MARKER`
- Return sequence → publishes `IDLE`

If the step's `motion_type` is `PAUSE`, execution halts and `_pause_handler(step, self._execute_next_step)` is invoked (or the sequence resumes immediately if no handler is set). Otherwise the step is dispatched to `_send_motion_goal`.

---

### `_send_motion_goal`

**Parameters**:
- `step`: `dict` — must contain `motion_type`; remaining keys are motion-specific (see [`task_modes`](./task_modes.md) and the [Motion package](../Motion/motion.md)).

**Return**: /

Builds an `ExecuteMotion.Goal` with `motion_type = step['motion_type']` and `step_data = json.dumps(step)` (the whole step is JSON-encoded), and sends it asynchronously. Goal acceptance and result are handled by inlined closures; `_on_motion_result` runs on completion. If the action server is not ready, or the goal is rejected, transitions to `ERROR`.

---

### `_on_motion_result`

**Parameters**:
- `future`: future returned by `get_result_async`.

**Return**: /

| Outcome | Behaviour |
| ---     | ---       |
| `result.success == True` | Stores `result.final_tcp_pose` (if present) into `_latest_tcp_pose`; calls `_execute_next_step` to continue. |
| `result.success == False` | Sets `_error_message`, clears the sequence and `_on_sequence_done`, transitions to `ERROR`. |
| Exception (server crash) | Logs, clears the sequence and `_on_sequence_done`, transitions to `ERROR`. |

---

### `_send_arduino_command`

**Parameters**:
- `msg_type`: `str` — command type (e.g. `'HAMMER_REQ'`, `'IND_VALUES'`).
- `data`: `str` — command payload.
- `msg_info`: `str` — optional metadata.
- `on_success`: `TaskState` or `callable` — state to publish, or function to call with `result.message`.
- `on_failure`: `TaskState` or `callable` — state to publish, or function to call (no args).

**Return**: /

Sends a generic `ArduinoCommand` action goal. On result, publishes or invokes `on_success` / `on_failure` depending on `result.success`. If the server is not ready, or the goal is rejected, transitions to `ERROR`.

---

### `_read_inductive`

**Parameters**:
- `sensor`: `int` — 1-based sensor index to test.
- `on_success` / `on_failure`: `TaskState` or `callable`.

**Return**: /

Sends `IND_VALUES|0|A` via `_send_arduino_command`. Parses the response `"<val_1>;<val_2>"` into `_inductive_states` and dispatches `on_success` if `values[sensor-1] == 1`, else `on_failure`. Parse errors fall through to `on_failure`. Wired up in `POSE_ACQUIRED` as the `_pause_handler`, reading sensor `2` with `_raise_inductive_error` on failure.
