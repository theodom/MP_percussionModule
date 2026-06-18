[back](../percussionModule.md#task-manager)
# Task manager package:

## Components

- Launchfile: [`task_system.launch`](./task_system_launch.md)
- Task Manager node: [`task_manager_node`](./task_manager_node.md)
- Task modes: [`task_modes`](./task_modes.md)
- Error handler: [`error_handler`](./error_handler.md)

## Goals

The task manager is the central orchestrator that coordinates perception, motion, percussion and arduino bridge subsystems to execute a complete hammering task from request to completion. The process is sequential, the task manager triggers the next step by updating the `task_manager/state` topic, which triggers a certain action/subsequence.

- Accept a hammering task request (mode `FIXING` / `LOOSENING`) via the `/start_task` ROS service
- Orchestrate subsystems:
    - Request the capture service to detect markers (retries once via the home sequence if none are found)
    - Always select the first detection (`detections[0]`) — no ranking logic yet
    - Build and execute mode-specific motion sequences (home, main, return) defined in [`task_modes`](./task_modes.md)
    - Read inductive sensors mid-sequence (via the `PAUSE` mechanism) to verify alignment
    - Request the hammer task to perform percussion
- Publish rich status on the `state` topic (`percussion_interfaces/msg/SystemState`)
- Classify failures and attempt automatic recovery through the [`error_handler`](./error_handler.md)

> The state machine is **event-driven**: each handler does its work and publishes the next state; the `state` subscriber (`_on_state_changed`) then dispatches the follow-up action. There is no central loop.
