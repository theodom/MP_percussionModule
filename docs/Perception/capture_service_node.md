# Capture service node

## Overview

ROS 2 service node. This node is responsible for all vision with an intel realsense camera. Receives capture (service) requests, triggers the camera pipeline and ArUco detection via [`detectAruco.py`](detectAruco.md), applies the camera-to-gripper transform, and returns the detected marker poses in Robot baseframe.

> Stateless between calls — no persistent camera connection is held.

## Components

- Node: `CaptureServiceNode`
    - `init`
    - `handle_capture`

## Goals

- Expose the `/trigger_capture` service to the rest of the system
- Run a full camera capture + ArUco detection cycle on request
- Convert each raw detection from camera frame to gripper/TCP frame via `cam2Gripper`
- Pack results into `MarkerDetection[]` and return them to the caller

## CaptureServiceNode

### `__init__`

**Parameters**:

/

**Return**:

/

**Declare Parameters**:

- `markerSize`: Physical side length of the ArUco markers in metres. Default: `0.0398`.

**Setup services**:

- `/trigger_capture` (`TriggerCapture`): Service that triggers the full capture pipeline. Callback: `handle_capture`.

---

### `handle_capture`

**Parameters**:

- `request`: `TriggerCapture.Request` — contains `timeout_sec` (logged but not enforced)
- `response`: `TriggerCapture.Response`

> timeout_sec is not reinforced, and service/pipeline timeout has never been an issue. could maybe be removed

**Return**:

- `response`: `TriggerCapture.Response`

Runs the full detection pipeline and populates the response. Two separate try/except blocks handle different failure points:

| Service result | Response message|
| ---   | ---              |
| Failure during camera initialisation           | `success=False`, `message='Failed connecting camera: {exception}'` |
| Failure in `Ar.detectMakrer` | `success=False`, `message='Failed during marker detection: {exception}'` | 
| Failure during pose transform or message formulation | `success=False`, `message='Failed during marker detection: {exception}'`|  
| No markers detected | `success=False`, `detections=[]`, `message='No markers in view'` |
| Markers detected | `success=True`, `detections=[MarkerDetection, ...]`, `message='Successfully detected markers'` |

For each detection the following is packed into a [`MarkerDetection`](../interfaces/msg/MarkerDetection.md) message:

- `marker_id`: integer ID from the ArUco detector
- `pose`: [`percussion_interfaces/Pose6D`](../interfaces/msg/Pose6D.md) with `x, y, z, rx, ry, rz` in the **gripper/TCP frame** (after `cam2Gripper`)


