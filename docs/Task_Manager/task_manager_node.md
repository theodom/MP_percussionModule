# Task_manager_node

### `init`:

**Declare Paramters**: 

- `capture_service_name`: Name of the service which handles camera image capture
- `target_frame`: unused
- `capture_timeout_sec`: Timeout in seconds for camera service, currently unimplemented, possibly redundant


**Setup services/topics**:
- `/start_task`: Service which triggers the system start: Callback calls the capture service.
- `/task_manager/state`: Publilisher for the current task state. IDLE/CAPTURING/...
-`_motion_client`: Create service client for the camera capture service.
-

### `_build_sequence`: 
**parameters**: 
 - marker_pose: Pose6D with marker coordinates in base_link frame.

**Return**: 
- List of Dicts containing motion type and movement instructions. 


| Key | Value | info |
| --- | ---   | ---  |
|`motion_type`| MOVE_TO_MARKER / RELATIVE_MOVE / MOVE_TO_CONTACT | type of motion corresponding to available types in percussion_motion package|
| `marker_pose` | `percussion_interfaces/ msg/Pose6D` | 6D TCP pose. Marker position in gripper frame for MOVE_TO_MARKER type. otherwise 6 x 0. |
| `approach_offset` | List of 6 floats. | relevant movement information for each move type. Offset for MOVE_TO_MARKER, direction + speed for MOVE_TO_CONTACT. relative final TCP Position for RELATIVE_MOVE.


> **Notice**: Entire build sequence concept/logic should be reworked to be more generalised. Including dictionary naming and content. 

### `_build_return_sequence`

**parameters**:

/

**Return**
- List of Dicts containing motion type and movement instructions. 

Logic is equal to `_build_sequence`, but containing different Dictionaries/motions. To be reworked and generalised along with said function.

### `publish_state`:

**Parameters:

- state: Enum TaskState containing current system state.

**Return:

/

| Value | info |
| ---   | ---  |
| IDLE | Default state at startup. Ready to accept `/start_task` service call. |
| TASK_REQUESTED | Service call received, state change published via pub/sub. |
| CAPTURING | Perception service in progress (waiting for vision capture). |
| POSE_ACQUIRED | Marker detected. Sequence building in progress. |
| MOVING_TO_WEDGELOCK | *(Deprecated state name, defined but not actively used).* |
| AT_MARKER | Main 10-step motion sequence complete. Awaiting percussion event (external/arduino trigger). |
| HAMMERING | Percussion event triggered. Percussion task in progress. |
| DONE | Percussion task completed. Ready to return to home. |
| RETURNING | Executing 3-step return sequence back to home position. |
| ERROR | Failure detected (capture timeout, motion failure, RTDE disconnect, etc.). See log for details. |


### start_task_callback

**Parameters**:

- Request: Trigger.Request
- Response: Trigger.Response

**Return**:
 - response: Trigger

Checks that the task manager is in a suitable state. Triggers the capture service and changes TaskState to CAPTURING. 

the start_callback also creates the link to the 'done_callback' upon service finish.

### `_on_capture_done`

**Parameters**:

- future: future object returned by the async capture service call.

**Return**:

/

Callback triggered when the capture service completes. Handles the following outcomes:

| Outcome | State transition | info |
| ---     | ---              | ---  |
| Exception / no result | ERROR | Service call itself failed. |
| `result.success == False` | ERROR | Perception node returned failure. |
| No markers detected | IDLE | Capture succeeded but scene had no markers. |
| Markers detected | POSE_ACQUIRED - (sequence start) | Logs all detections, selects `detections[0]`, builds the motion sequence and calls `_execute_next_step`. |

> Possible improvement: if no markers detected, request robot move/rotation and retry instead of returning to IDLE.

> Always picks `detections[0]`. No ranking or filtering logic exists.

---

### `_execute_next_step`

**Parameters**:

/

**Return**:

/

Pops the first step from the active sequence list and dispatches it to `_send_motion_goal`.

If the sequence is empty:
- If the **main sequence** just finished (`_returning == False`): sets state to DONE, loads the return sequence, sets `_returning = True`, and immediately starts executing it.
- If the **return sequence** just finished (`_returning == True`): sets state to IDLE and returns.

This means the full task cycle runs as a self-driven chain of callbacks with no timer or thread involvement.

---

### `_send_motion_goal`

**Parameters**:

- step: Dict with keys `motion_type`, `marker_pose`, `approach_offset`. One element from the sequence list.

**Return**:

/

Constructs an `ExecuteMotion.Goal` from the step dict and sends it asynchronously to the `/execute_motion` action server. Sets state to `MOVING_TO_WEDGELOCK` or `RETURNING` depending on `_returning`. Registers `_on_motion_goal_accepted` as the done callback.

If the action server is not ready, immediately transitions to ERROR.

---

### `_on_motion_goal_accepted`

**Parameters**:

- future: future returned by `send_goal_async`.

**Return**:

/

Checks whether the motion node accepted the goal. If rejected, transitions to ERROR. If accepted, registers `_on_motion_result` to be called when the action finishes.

---

### `_on_motion_result`

**Parameters**:

- future: future returned by `get_result_async`.

**Return**:

/

Handles the result of a completed motion step.

| Outcome | Behaviour |
| ---     | ---       |
| `result.success == True` | Calls `_execute_next_step` to continue the sequence. |
| `result.success == False` | Logs the error message, clears the remaining sequence, transitions to ERROR. |

