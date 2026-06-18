[back](../percussionModule.md)
# Reconnect service

## Overview

Recovery service served by the [`percussion_motion_node`](../../Motion/percussion_motion_node.md) at `/percussion/motion/reconnect`. It closes and re-opens the RTDE connection to the UR10e. The request is empty; the response reports whether the reconnect succeeded.

Called by the task manager's [`error_handler`](../../Task_Manager/error_handler.md) when a failure is classified as `ROBOT_CONNECTION`: on success the handler resumes the in-flight step by re-publishing the pre-`ERROR` state.

## Content

```
---
bool success
string message
```

| Field | info |
| ---   | ---  |
| `success` | `True` if RTDE was re-established. |
| `message` | Status / failure detail. |
