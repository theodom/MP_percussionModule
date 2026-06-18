[back](./task_manager.md)
# percussionModule Launch file

## Overview

Single launchfile (`task_system.launch.py`) of the [Task manager](./task_manager.md) package. It starts [`task_manager_node`](./task_manager_node.md) and all underlying nodes, plus the static TF broadcasters. All percussion nodes are pushed under the `percussion/` namespace; the TF broadcasters are deliberately kept **outside** the namespace (`/tf`, `/tf_static` are global).

It exposes a set of launch arguments overridable at launch (e.g. `robot_ip`, `default_velocity`).

## Launch arguments

| Argument | Default | info |
| ---      | ---     | ---  |
| `marker_size` | `0.0398` | ArUco marker side length (m). → perception. |
| `robot_ip` | `169.254.0.22` | UR10e IP over RTDE. → motion. |
| `default_velocity` | `1.0` | Default Cartesian velocity (m/s). → motion. |
| `default_accel` | `0.50` | Default Cartesian acceleration (m/s²). → motion. |
| `contact_force` | `5.0` | Contact-detection force threshold (N). → motion. |
| `contact_timeout` | `5.0` | Contact-detection timeout (s). → motion. |
| `arduino_port` | `/dev/ttyACM0` | Arduino serial port. → arduino bridge. |
| `arduino_baudrate` | `115200` | Arduino serial baudrate. → arduino bridge. |

## Components

- [**task_manager_node**](./task_manager_node.md) (`task_manager` namespace): high-level orchestrator. No launch parameters.

- [**perception_node**](../Perception/perception.md) (`perception` namespace): vision service server.
    - `markerSize` ← `marker_size`

- [**percussion_motion_node**](../Motion/percussion_motion_node.md) (`motion` namespace): robot motion action server.
    - `robot_ip`, `default_velocity`, `default_accel`, `contact_force`, `contact_timeout`

- [**arduino_bridge_node**](../arduino_bridge/arduino_bridge_node.md) (`arduino_bridge` namespace): serial bridge to the Arduino.
    - `port` ← `arduino_port`, `baudrate` ← `arduino_baudrate`

- **static_transform_publisher** (`tool0_to_camera_broadcaster`): static transform `tool0` → `camera_frame` (`[0, -0.12775, 0.04451]`, yaw `3.14`).

- **static_transform_publisher** (`world_to_base_link_broadcaster`): static transform `world` → `base_link` (45° / `0.785398` rad yaw — the robot mounting rotation).
