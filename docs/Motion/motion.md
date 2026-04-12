# Motion package

## Components

- [`percussion_motion_node`](./percussion_motion_node.md)
- [`rtde_motions`](./rtde_motions.md)


## Goals

Action server which takes a [`ExecuteMotion.goal`](../interfaces/action/ExecuteMotion.md) as input parameters. Depending on the `goal.motion_type` it orchestrates the correct robot movement from [`rtde_motions`](./rtde_motions.md) 
It returns a `ExecuteMotion.Result` object to the action client ([`task_manager_node`](../Task_Manager/task_manager_node.md))s

### Percussion_motion_node   




### rtde_motions

