import pyrealsense2 as rs
import cv2
import cv2.aruco as aruco
import numpy as np

# # --- Setup RealSense ---
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# profile = pipeline.start(config)
# 
# align = rs.align(rs.stream.color)
# 
# # --- Get intrinsics directly from the camera (no chessboard needed) ---
# intrinsics = profile.get_stream(rs.stream.color) \
#                     .as_video_stream_profile().get_intrinsics()
# 
# mtx = np.array([[intrinsics.fx, 0,             intrinsics.ppx],
#                 [0,             intrinsics.fy, intrinsics.ppy],
#                 [0,             0,             1            ]])
# dist = np.array(intrinsics.coeffs)



# --- ArUco setup ---
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())

marker_size = 0.07  # your measured marker size in meters

marker_points_3d = np.array([
    [-marker_size/2,  marker_size/2, 0],
    [ marker_size/2,  marker_size/2, 0],
    [ marker_size/2, -marker_size/2, 0],
    [-marker_size/2, -marker_size/2, 0]], dtype=np.float32)


def initialise_camera():
    """
    Initialize the Intel RealSense camera.

    Returns:
        pipeline (rs.pipeline): The configured RealSense pipeline.
    """
    print("entering camera init")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    print("exiting camera init")
    return pipeline, profile


def rvec_to_euler(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        roll  = np.degrees(np.arctan2( R[2,1], R[2,2]))
        pitch = np.degrees(np.arctan2(-R[2,0], sy))
        yaw   = np.degrees(np.arctan2( R[1,0], R[0,0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1,2], R[1,1]))
        pitch = np.degrees(np.arctan2(-R[2,0], sy))
        yaw   = 0
    return roll, pitch, yaw


def draw_info_overlay(frame, ids, corners, rvecs, tvecs, points_3d, profile):
    _, mtx, dist, _ = getMatrices(profile)

    for i in range(len(ids)):
        corner = corners[i][0]
        pt3d   = points_3d[i]
        roll, pitch, yaw = rvec_to_euler(rvecs[i])

        # Marker outline + center
        cv2.polylines(frame, [corner.astype(int)], True, (0, 255, 0), 2)
        cx = int(corner[:, 0].mean())
        cy = int(corner[:, 1].mean())
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

        # 3D axes
        cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], marker_size * 0.5)

        # Info box
        lines = [
            ("ID",    f"ID: {ids[i][0]}",         (0,   255, 255)),
            ("X",     f"X: {pt3d[0]*100:+.1f} cm",(255, 100, 100)),
            ("Y",     f"Y: {pt3d[1]*100:+.1f} cm",(100, 255, 100)),
            ("Z",     f"Z: {pt3d[2]*100:+.1f} cm",(100, 100, 255)),
            ("Roll",  f"Roll:  {roll:+.1f} deg",   (200, 200,  50)),
            ("Pitch", f"Pitch: {pitch:+.1f} deg",  (200, 200,  50)),
            ("Yaw",   f"Yaw:   {yaw:+.1f} deg",   (200, 200,  50)),
        ]

        x0     = int(corner[:, 0].min())
        y0     = int(corner[:, 1].min())
        box_h  = len(lines) * 18 + 10
        y_start = max(y0 - box_h - 10, 0)

        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y_start),
                      (x0 + 165, y_start + box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        for j, (_, text, color) in enumerate(lines):
            cv2.putText(frame, text, (x0 + 5, y_start + 15 + j * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    return frame

# nodig: intrinsics, mtx, dist, pipeline, align

def getMatrices(profile):
    align = rs.align(rs.stream.color)
    # --- Get intrinsics directly from the camera (no chessboard needed) ---
    intrinsics = profile.get_stream(rs.stream.color) \
                        .as_video_stream_profile().get_intrinsics()

    mtx = np.array([[intrinsics.fx, 0,             intrinsics.ppx],
                    [0,             intrinsics.fy, intrinsics.ppy],
                    [0,             0,             1            ]])
    dist = np.array(intrinsics.coeffs)

    return intrinsics, mtx, dist, align

def detectMarker(pipeline, profile, markerSize=0.07, verbose = False):

    intrinsics, mtx, dist, align = getMatrices(profile)
    frames  = pipeline.wait_for_frames()
    align = rs.align(rs.stream.color)
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        return []

    color_image = np.asanyarray(color_frame.get_data())
    corners, ids, _ = detector.detectMarkers(color_image)

    results = []
    if ids is not None:
        rvecs, tvecs, points_3d = [], [], []

        for corner in corners:
            # 3D position from depth
            cx = int(corner[0][:, 0].mean())
            cy = int(corner[0][:, 1].mean())
            depth = depth_frame.get_distance(cx, cy)
            points_3d.append(
                rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth))

            # Orientation from solvePnP
            _, rvec, tvec = cv2.solvePnP(
                marker_points_3d, corner[0], mtx, dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(rvec)
            tvecs.append(tvec)

            # Combine into [x, y, z, rx, ry, rz] — all relative to camera
            pose = np.concatenate([tvec.flatten(), rvec.flatten()])
            results.append(pose)

        color_image = draw_info_overlay(
            color_image, ids, corners, rvecs, tvecs, points_3d, profile)
    if verbose:
        cv2.imshow('ArUco D405', color_image)
        if cv2.waitKey(1) == ord('q'):  # ESC to quit
            cv2.destroyAllWindows()
    return results, ids

def cam2Gripper(pose_cam):
    #R_cam2gripper = np.array([
    #    [-0.98263,  0.15689,  0.09911],
    #    [-0.15022, -0.98606,  0.07153],
    #    [ 0.10896,  0.0554 ,  0.9925 ]
    #])
    #T_cam2gripper = np.array([[-0.01776],[ 0.11886],[ 0.05126]])
    R_cam2gripper = np.array([
        [-1.,  0., 0.],
        [ 0., -1., 0.],
        [ 0.,  0., 1.]
    ])
    T_cam2gripper = np.array([[0], [0.12775], [-0.04451]])

    H_cam2gripper = np.eye(4)
    H_cam2gripper[:3, :3] = R_cam2gripper
    H_cam2gripper[:3, 3:] = T_cam2gripper

    # Build 4x4 pose matrix for the marker in camera frame
    R_marker_cam, _ = cv2.Rodrigues(np.array(pose_cam[3:]))
    H_marker_cam = np.eye(4)
    H_marker_cam[:3, :3] = R_marker_cam
    H_marker_cam[:3, 3] = pose_cam[:3]

    # Single matrix multiply does both position and orientation
    H_marker_gripper = H_cam2gripper @ H_marker_cam

    # Extract position and rotation vector back out
    pos = H_marker_gripper[:3, 3]
    rvec, _ = cv2.Rodrigues(H_marker_gripper[:3, :3])

    return np.concatenate([pos, rvec.flatten()])
