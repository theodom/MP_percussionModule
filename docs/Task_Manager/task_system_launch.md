# percussionModule Launch file

## Overview

Single launchfile part of [Task manager](./task_manager.md) package, wich starts [`task_manger_node`](./task_manager_node.md) and all underlying nodes, as well as other auxiliary components.
It exposes a set of parameters changeable at launch, such as `robot_ip`, `default_vel` for the robot motion, etc.


## Components

- [**task_manager_node**](./task_manager_node.md): High level orchestrator node.
    - **parameters**
    - `capture_timeout_sec`: float value for the capture service timeout.

- [**capture_service_node**](../Perception/capture_service_node.md): Service server for the vision system. 
    - **parameters**
    - `marker_size`: float value for the physical size of the to be detected ArUco markers. 

- [**percussion_motion_node**](../Motion/percussion_motion_node.md): Action server for the robot motion system.
    - **parameters**
    - `robot_ip`: string for the IP address of the ur10e Robot arm.       
    - `default_velocity`: float value for the default robot velocity in m/s.
    - `default_accel`: float value for the default robot acceleration in m/s².   
    - `contact_force': float value for the contact threshold in N.
    - `contact_timeout': float value for the duration before 
    - moveToContact sequence times out and quits.

- **static_transform_publisher**: Static transformation `camera_frame` -> `tool0` or the robot TCP frame.

- **static_transform_publisher**: Static transformation for the 45° rotation of the base link relative to the world frame (robot mounting position).

