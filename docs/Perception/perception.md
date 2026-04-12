# Perception package

## Components

- Capture service node: [`capture_service_node.py`](capture_service_node)
- ArUco detection module: [`detectAruco.py`](detectAruco)

## Goals

- Accept capture requests from the task manager
- Capture a single RGB frame from the Intel RealSense camera
- Detect ArUco markers in the color image
- Compute the 3D pose of each detected marker in the camera frame
- Transform detected poses from camera frame to gripper/TCP frame
- Return all detections as [`percussion_interfaces/MarkerDetection[]`](../interfaces/msg/MarkerDetection.md) to the caller.

> Notice: Structure of Service response with List of custom messages of custom messages could maybe be simplified to list of Pose6D + list of markerID's (int)?


### capture_service_node

Single-service ROS 2 node. Exposes [`/trigger_capture`](../interfaces/srv/TriggerCapture.md) and delegates all camera and detection logic to [`detectAruco.py`](./detectAruco.md). Stateless between calls — no persistent camera connection is maintained.

**Parameters**:

- `markerSize`: Physical side length of the ArUco markers in metres. Default: `0.0398`.

**Services**:

- `/trigger_capture` (`TriggerCapture`): Receives a capture request, runs the full detect pipeline, and returns `MarkerDetection[]` in the gripper frame.

> `request.timeout_sec` is received and logged but not enforced — the capture is not time-bounded. 
>>To be implemented or removed. So far timeout has never been an issue.


### detectAruco

Pure-Python module (no ROS). Contains all camera and vision logic. Called directly by `capture_service_node`.

**Camera**:
- Intel RealSense D-series (depth + color, only color is used)
- Resolution: 640 × 480, 30 fps
- Depth stream aligned to color stream on every capture
- Camera intrinsics are read live from the device — no calibration file needed
- Pipeline is started and stopped on every call to `initialise_camera` / `pipeline.stop()` — no persistent connection

**Detection**:
- ArUco dictionary: `DICT_4X4_50`
- Detector: `cv2.ArucoDetector` with default parameters
- 3D position: depth at marker center pixel via `rs2_deproject_pixel_to_point`
- Orientation: `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE` flag, returns Rodrigues vector

**Camera-to-gripper transform** (`cam2Gripper`):

Applied via a 4×4 homogeneous matrix multiply. Hardcoded values:

| | Value | Info |
| --- | --- | --- |
| Rotation | 180° flip in X and Y (`diag([-1, -1, 1])`) | Align camera axes with TCP axes
| Translation | `[0, 0.12775, -0.04451]` m | Translation from camera center to TCP center, as measured from cad file. 

The result is a 6D pose `[x, y, z, rx, ry, rz]` in the gripper/TCP frame, with orientation expressed as a Rodrigues vector.

> The rotation matrix is hardcoded. eye-in-hand calibration was implemented but did not give acceptably accurate results.
