[back](../percussionModule.md)
# SystemState message

## Goal

Rich status message published by the [`task_manager_node`](../../Task_Manager/task_manager_node.md) on its `state` topic. It replaces the earlier plain `std_msgs/String` state so a single message carries the full system snapshot (task progress, mode, sensor readings, TCP pose, last error and detected marker). Published on every state transition and once per second as a heartbeat.

## Content

```
string task_state
string task_mode
int32[] inductive_states
Pose6D tcp_pose
string error_message
int32 detected_marker_id
```

| Field | info |
| ---   | ---  |
| `task_state` | Current `TaskState` value (`IDLE`, `CAPTURING`, …). |
| `task_mode` | Active mode name from the `MODES` registry (`FIXING` / `LOOSENING`), or empty. |
| `inductive_states` | Last parsed `IND_VALUES` reading. |
| `tcp_pose` | Latest TCP pose ([`Pose6D`](./Pose6D.md)) from the most recent motion result. |
| `error_message` | Last error string (cleared on `IDLE`). |
| `detected_marker_id` | Marker id from the last successful capture (`-1` when none). |
