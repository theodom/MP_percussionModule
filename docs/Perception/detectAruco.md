# detectAruco.py

## Overview

Pure-Python vision module. No ROS dependency. Contains all camera initialisation and ArUco marker detection logic. Called directly by `capture_service_node`.

## Components

- `initialise_camera`: Start the RealSense pipeline
- `getMatrices`: Extract camera intrinsic matrix and distortion coefficients from a live profile
- `detectMarker`: Core detection function — capture a frame, detect markers, compute 3D poses
- `cam2Gripper`: Transform a detected pose from camera frame to gripper/TCP frame
- `rvec_to_euler`: Convert Rodrigues rotation vector to Euler angles (roll, pitch, yaw) — used for visualisation only
- `draw_info_overlay`: Draw detected marker info onto a frame — used for visualisation only

---

### `initialise_camera`

**Parameters**:

/

**Return**:

- `pipeline`: `rs.pipeline` - active RealSense pipeline
- `profile`: pipeline profile, used to extract intrinsics via `getMatrices`

Starts the RealSense pipeline with both color and depth streams at 640×480, 30 fps. Called once per capture request; the caller is responsible for calling `pipeline.stop()` afterwards.

>Potential upgrade: use 720p or 1080p image. 

> Camera is not kept alive between requests — every call to `initialise_camera` does a full device open/close cycle. This adds latency but avoids holding the USB device between tasks.

---

### `getMatrices`

**Parameters**:

- `profile`: Active pipeline profile returned by `initialise_camera`

**Return**:

- `intrinsics`: RealSense intrinsics object (used by `rs2_deproject_pixel_to_point`)
- `mtx`: 3×3 numpy camera matrix `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`
- `dist`: distortion coefficients array from the device
- `align`: `rs.align` object configured to align depth to color

Intrinsics are read live from the device on every call — no external calibration file is used.

---

### `detectMarker`

**Parameters**:

- `pipeline`: Active `rs.pipeline`
- `profile`: Active pipeline profile
- `markerSize`: Marker side length in metres. Default: `0.0398`
- `verbose`: If `True`, displays the annotated frame in a window via `cv2.imshow`. Default: `False`

**Return**:

- `results`: List of `np.array([x, y, z, rx, ry, rz])` — one per detected marker, in **camera frame**
- `ids`: List of marker IDs corresponding to each result

Captures a single aligned RGB-D frame and runs the full detection pipeline:

1. Align depth to color frame
2. Run `cv2.ArucoDetector.detectMarkers` on the color image
3. For each detected marker:
   - Compute center pixel `(cx, cy)` from corner coordinates
   - Read depth at center via `depth_frame.get_distance(cx, cy)`
   - Deproject to 3D point using `rs2_deproject_pixel_to_point` — gives `[x, y, z]` in camera frame
   - Run `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE` to get orientation as Rodrigues vector `[rx, ry, rz]`
   - Concatenate into `[x, y, z, rx, ry, rz]`
4. Return all results and corresponding IDs

> The returned poses are in the **camera frame**, not the gripper frame. The caller (`capture_service_node`) applies `cam2Gripper` afterwards.

> Depth is sampled at a single center pixel. Noisy depth readings (e.g. at edges or shiny surfaces) will directly affect position accuracy.

---

### `cam2Gripper`

**Parameters**:

- `pose_cam`: `np.array([x, y, z, rx, ry, rz])` — marker pose in camera frame, orientation as Rodrigues vector

**Return**:

- `np.array([x, y, z, rx, ry, rz])` — marker pose in gripper/TCP frame, orientation as Rodrigues vector

Applies a fixed camera-to-gripper rigid body transform using a 4×4 homogeneous matrix multiply:


Hardcoded transform values:

| | Value |
| --- | --- |
| Rotation | 180° flip around Z axis: `diag([-1, -1, 1])` |
| Translation | `[0, 0.12775, -0.04451]` m |

> A previously commented-out block contains a calibrated rotation matrix from an earlier hand-eye calibration run. The current matrix is a simplified approximation. If the camera mount is adjusted, this transform must be recalibrated.

