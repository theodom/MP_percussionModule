import rclpy
from rclpy.node import Node

from percussion_interfaces.srv import TriggerCapture
from percussion_interfaces.msg import MarkerDetection
from percussion_interfaces.msg import Pose6D

import percussion_perception.detectAruco as Ar


class CaptureServiceNode(Node):
    def __init__(self):
        super().__init__('capture_service_node')

        self._srv = self.create_service(
            TriggerCapture,
            '/trigger_capture',
            self.handle_capture
        )

        self.declare_parameter('markerSize', 0.0398)

        self._markerSize = self.get_parameter('markerSize').get_parameter_value().double_value
        self.get_logger().info('Capture service node ready.')

    def handle_capture(self, request, response):
        self.get_logger().info(f'Received capture request: timeout={request.timeout_sec}')

        try:
            pipeline, prof = Ar.initialise_camera()
            detections, ids = Ar.detectMarker(pipeline, prof, markerSize=self._markerSize)
            pipeline.stop()

        except Exception as e:
            response.success = False
            response.detections = []
            response.message = f'Failed connecting camera: {e}'
            return response

        try: 
            if len(detections) > 0:

                response.detections = []

                pairs = list(zip(detections, ids))

                for det, id_ in pairs:
                    
                    det_gripper = Ar.cam2Gripper(det)
                    

                    msg = MarkerDetection()

                    msg.marker_id = int(id_)

                    pose = Pose6D()

                    pose.x = det_gripper[0]
                    pose.y = det_gripper[1]
                    pose.z = det_gripper[2]

                    pose.rx = det_gripper[3]
                    pose.ry = det_gripper[4]
                    pose.rz = det_gripper[5]
                    msg.pose = pose

                    response.detections.append(msg)
                response.success = True
                response.frame = "camera_frame"
                response.message = 'Successfully detected markers'
            else:
                response.success = False
                response.detections = []
                response.message = 'No markers in view'

            return response
        except Exception as e:
            response.success = False
            response.detections = []
            response.message = f'Fault during processing: {e}'
            return response



def main(args=None):
    rclpy.init(args=args)
    node = CaptureServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()