# Percussion Module — ROS 2 Workspace

ROS 2 (Jazzy) workspace for a robotic percussion/scaffolding fixation system. A UR10e arm detects ArUco markers via an Intel RealSense camera and moves to them.

---

## Setup & Usage

### 1. Start the UR robot driver

The `ur_robot_driver` must be running before launching this system. It is a separate ROS 2 stack and is **not** included in this workspace.

**Simulation (URSim):**
```bash
# Terminal 1 — start the simulated teach pendant
ros2 run ur_client_library start_ursim.sh -m ur10e
```

```bash
# (optional) Terminal 2 — start the driver (connects to URSim at 192.168.56.101)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=true
```

**Real robot** (IP `169.254.0.22`, link-local):
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e robot_ip:=169.254.0.22 launch_rviz:=false
```

### 2. Launch the percussionModule task manager

```bash
ros2 launch percussion_task_manager task_system.launch.py
```

### 3. Trigger a task (system: 'start')

```bash
ros2 service call /start_task std_srvs/srv/Trigger
```

---

## Architecture

Three nodes are started by the launch file, along with two static TF publishers (`world→base_link`, `tool0→camera_frame`).

```
/start_task (Trigger)
      │ 
      ▼
task_manager_node
      │  /trigger_capture (TriggerCapture srv)
      ▼
capture_service_node          ← RealSense + ArUco detection
      │
      │ MarkerDetection[] (pose in gripper frame)
      ▼
task_manager_node
      │  /execute_motion (ExecuteMotion action)
      ▼
percussion_motion_node        ← RTDE → UR10e
```

| Node | Package | Role |
|---|---|---|
| `capture_service_node` | `percussion_perception` | Opens RealSense, detects ArUco markers, returns poses in gripper frame |
| `task_manager_node` | `percussion_task_manager` | Orchestrator state machine: calls perception then sends motion goal |
| `percussion_motion_node` | `percussion_motion` | Executes motion primitives on the UR10e via RTDE |

**Custom interfaces** (`percussion_interfaces`): `msg/Pose6D`, `msg/MarkerDetection`, `srv/TriggerCapture`, `action/ExecuteMotion`.

**State machine** (`task_manager_node`): `IDLE → CAPTURING → POSE_ACQUIRED → MOVE_TO_MARKER → DONE / ERROR`

---

## TO DO

### Currently known issues

- **`perception_motion`**: 
      - The robot currently moves using rtde, but to the wrong position -- problem with marker pose transform camera -> TCP pose
 -    Add `MOVE_HOME`

- **Marker selection**: always picks `detections[0]`. Should select by lowest marker ID or sequentially from the last struck marker. (Current plan unknown)

- **`capture_service_stub.py`**: should be removed.

- **Shutdown warning**: all three nodes emit `rcl_shutdown already called` — deduplicate `rclpy.shutdown()` calls in launch.

### Features to add

- **Tool control**:
      - Write arduino program to read sensors and control actuators
      - Add tool control logic
      - Serial communication / Arduino-ROS bridge
      - tool controller node
      - ...

- **Move until contact**
      - Implement move until contact: requires bridge between rtde, Arduino serial and ROS



