[back](../percussionModule.md)
# StartTask service

## Overview

Entry point that triggers a percussion task. The request carries the task `mode` (case-insensitive, validated against the `MODES` registry in [`task_modes`](../../Task_Manager/task_modes.md)). On a valid mode the [`task_manager_node`](../../Task_Manager/task_manager_node.md) stores it and publishes `TASK_REQUESTED`; an unknown mode is rejected with `success=False` and the list of valid modes, with no state change.

## Content

```
string mode
---
bool success
string message
```

| Field | info |
| ---   | ---  |
| `mode` | Requested task mode — `"FIXING"` or `"LOOSENING"`. |
| `success` | `True` if the mode was accepted. |
| `message` | Confirmation, or the rejection reason / valid modes. |
