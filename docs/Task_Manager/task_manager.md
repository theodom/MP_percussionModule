[back](../percussionModule.md#task-manager)
# Task manager package:

## Components

- Launchfile: [`task_system.launch`](./task_system_launch.md)
- Task Manager node: [`task_manager_node`](task_manager_node)

## Goals

The task manager is the central orchestrator that coordinates perception, motion, percussion and arduino bridge subsystems to execute a complete hammering task from request to completion. The process is sequential, the task manager triggers the next step by updating the `task_manager/state` topic, which triggers a certain action/subsequence. 

- Accept hammering task request via ROS service
- Orchestrate subsystems:
    - Request capture service to detect markers
    - Decide on marker to select from detections
    - Initiate movement sequences (approach, contact, repositioning)
    - Request hammer task to perform percussion


