# Percussion Module — ROS 2 Workspace

ROS 2 (Jazzy) workspace for the percussion Module of the PERI UP Genio mobile platform system. 

1. The percussion module detects a wedgelock position based on ArUco markers via an Intel RealSense camera and moves to them with the ur10e robot arm via RTDE. 

2. robot arm moves towards the ledger in 1D until contact, Move tool upwards of ledger, repeat move until contact downwards. Move over ledger pin. Then move until contact in 3rd dimension. Wedgelock position is now known.

3. Once positioned over wedgelock, Hammer action is initiated through arduino_bridge node.

4. Robotarm moves back to home position.

---

## Setup & Usage

### Dependencies

**System libraries** (install via apt):
```bash
sudo apt install librealsense2-dev librealsense2-utils
```

**ROS 2 packages** (install via apt):
```bash
sudo apt install ros-jazzy-ur ros-jazzy-ur-robot-driver ros-jazzy-tf2-ros
```

**Python packages** (install via pip):
```bash
pip install ur_rtde pyrealsense2 opencv-python pyserial numpy --break-system-packages
```


### 1. Launch the percussionModule task manager

```bash
ros2 launch percussion_task_manager task_system.launch.py
```

### 2. Trigger a task (system: 'start')

```bash
ros2 service call /percussion/task_manager/start_task percussion_interfaces/srv/StartTask "mode: FIXING"
```
or 
```bash
ros2 service call /percussion/task_manager/start_task percussion_interfaces/srv/StartTask "mode: LOOSENING"
```

---

## Architecture

The following diagram shows the architectural layout of the ROS project. The task manager node is a main orchestrator which decides the system state and what to do. The underlying nodes are called upon as needed;

![ROS node architecture](docs/ros_node_architecture.svg)


The following diagram gives an overview of the general workflow of the program. When a task is requested this is the sequence in which things are executed. (not yet fully implemented)

![ROS task flow sequence](docs/ros_task_flow_sequence.svg)

Four nodes are started by the launch file, along with two static TF publishers (`world` -> `base_link` and `tool0` -> `camera_frame`).




| Node | Package | Role |
|---|---|---|
| `perception_node` | `percussion_perception` | Opens RealSense, detects ArUco markers, returns poses in gripper frame |
| `task_manager_node` | `percussion_task_manager` | Orchestrator state machine: calls perception then sends motion goal |
| `motion_node` | `percussion_motion` | Executes motion primitives on the UR10e via RTDE |
| `arduino_bridge_node` | `percussion_arduino_bridge`| Interfaces with arduino through serial communication and interacts with ROS through actions.  

**Custom interfaces** (`percussion_interfaces`): `msg/Pose6D`, `msg/MarkerDetection`, `msg/SystemState`, `srv/TriggerCapture`, `srv/StartTask`, `srv/Reconnect`, `action/ExecuteMotion`, `action/ArduinoCommand`.

**State machine** (`task_manager_node`): 
`IDLE -> TASK_REQUESTED -> AT_HOME -> CAPTURING -> POSE_ACQUIRED -> AT_MARKER -> HAMMERING -> DONE -> RETURNING -> IDLE`
(`ERROR` is reachable from any state on failure, and is handled by the error handler.)

---

