[back](./task_manager.md)
# error_handler

## Overview

Centralised failure handling for the [`task_manager_node`](./task_manager_node.md). Every failure in the task manager funnels through `TaskState.ERROR` with a human-readable string in `_error_message`. The `ERROR` handler calls `ErrorHandler.handle_error(error_message, context)`, which **classifies** the string into a category, **chooses** a recovery action (subject to a per-category retry budget), and **dispatches** it.

`ErrorHandler` holds a back-reference to the node, so each recovery primitive can drive the state machine, re-issue service calls, and queue new motion sequences through the existing node API. It also owns a client for the motion node's [`Reconnect`](../interfaces/srv/Reconnect.md) service.

## Pipeline

```
handle_error(message, context)
   |
   |-- _classify(message)        -> ErrorCategory      (first matching keyword row)
   |-- _choose_action(category)  -> RecoveryAction      (apply retry budget; escalate to SAFETY_STOP)
   |-- _log(...)
   `-- _dispatch(action, ...)    -> recovery primitive
```

## Categories & classification

`_classify` upper-cases the message and walks an **ordered** table (`_CLASSIFICATION_TABLE`) of `(keywords, category)` rows; the first row whose keywords **all** appear wins, else `UNKNOWN`. More specific rows come first (motion *timeouts* before generic motion *failures*).

| `ErrorCategory` | Example keywords |
| ---             | ---              |
| `MOTION_TIMEOUT` | `CONTACT`+`TIMEOUT`, `FORCE`+`TIMEOUT`, `TIMEOUT` |
| `MARKER_NOT_FOUND` | `NO MARKERS`, `MARKER`+`NOT`, `VISION` |
| `CAMERA_FAULT` | `REALSENSE`, `CAMERA`, `CAPTURE` |
| `ROBOT_CONNECTION` | `RTDE`, `ROBOT`+`CONNECT`, `MOTION`+`SERVER`, `MOTION`+`NOT ACCEPTED` |
| `INDUCTIVE_FAULT` | `INDUCTIVE`, `IND_VALUES` |
| `ARDUINO_FAULT` | `ARDUINO`, `HAMMER`, `SERIAL` |
| `MOTION_FAILURE` | `MOTION` (catch-all) |
| `CONFIG_FAULT` | `MODE`, `CONFIG` |
| `UNKNOWN` | *(no match)* |

## Policy: action + retry budget

`CATEGORY_TO_ACTION` gives the first-line action per category; `MAX_RETRIES` is the per-category budget. `_choose_action` escalates to `SAFETY_STOP` once `attempts >= budget`. Counters are keyed by category and cleared by `reset_retries()` (called when the state machine returns to `IDLE`).

| Category | First-line action | Max retries |
| ---      | ---               | ---         |
| `MARKER_NOT_FOUND` | `MOVE_HOME_AND_RETRY` | 2 |
| `CAMERA_FAULT` | `RECONNECT_AND_RETRY` | 2 |
| `ROBOT_CONNECTION` | `RECONNECT_AND_RETRY` | 2 |
| `MOTION_TIMEOUT` | `MOVE_HOME_AND_RETRY` | 2 |
| `MOTION_FAILURE` | `SAFETY_STOP` | 0 |
| `INDUCTIVE_FAULT` | `MOVE_HOME_AND_RETRY` | 2 |
| `ARDUINO_FAULT` | `RECONNECT_AND_RETRY` | 2 |
| `CONFIG_FAULT` | `RETURN_TO_IDLE` | 0 |
| `UNKNOWN` | `SAFETY_STOP` | 0 |

## Recovery actions

| `RecoveryAction` | Behaviour |
| ---              | ---       |
| `MOVE_HOME_AND_RETRY` | Runs the active mode's **return** sequence (safe retract from any in-task pose), then publishes `TASK_REQUESTED` so the normal home + capture path re-runs. Falls back to `SAFETY_STOP` if no mode is active. |
| `RECONNECT_AND_RETRY` | Category-specific (`_attempt_reconnect`): see below. |
| `RETRY_CAPTURE` | Re-issues `run_capture_service()` from the current pose. |
| `RETURN_TO_IDLE` | Clears pending work and parks in `IDLE` without moving the robot (config faults where retrying is pointless). |
| `SAFETY_STOP` | Clears the pending sequence / handlers and publishes `IDLE` for manual intervention. Recover via `/start_task`. |
| `NONE` | No-op. |

**`RECONNECT_AND_RETRY` paths** (`_attempt_reconnect`):
- `ROBOT_CONNECTION` → calls [`/percussion/motion/reconnect`](../interfaces/srv/Reconnect.md) async; on success resumes via `_resume_after_reconnect`. If the service is not ready, waits then resumes anyway.
- `CAMERA_FAULT` → waits `_USB_RETRY_DELAY_SEC` (10 s), then re-triggers capture (perception opens a fresh RealSense pipeline per capture).
- `ARDUINO_FAULT` → waits 10 s, then resumes (arduino_bridge opens the serial port per command).

## Resume mechanism

`_resume_after_reconnect` re-publishes the pre-`ERROR` state (`context['previous_state']`, falling back to the node's `_previous_state`) so the failed step re-issues. Because the node's `_on_state_changed` dedupes on `_last_processed_state` (currently `'ERROR'`), it first forces `_last_processed_state = 'ERROR'` so the resume publish actually fires the handler. Falls back to `MOVE_HOME_AND_RETRY` if the previous state is missing or unrecognised.

`_wait_then(seconds, fn)` schedules `fn()` once via a one-shot rclpy timer — non-blocking, so the executor is never stalled during the USB/serial retry delays.

## Adding a new error category

1. Add a value to `ErrorCategory`.
2. Add a `(keywords, category)` row to `_CLASSIFICATION_TABLE` (first match wins — order matters).
3. Add rows to `CATEGORY_TO_ACTION` and `MAX_RETRIES`.
4. For a brand-new action, add a `RecoveryAction` value and an `elif` branch in `_dispatch`.
