import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from percussion_interfaces.msg import MarkerDetection
from percussion_interfaces.srv import TriggerCapture


class CaptureServiceStub(Node):
    def __init__(self) -> None:
        super().__init__('capture_service_stub')

        self._srv = self.create_service(
            TriggerCapture,
            '/trigger_capture',
            self.handle_capture,
        )

        self.get_logger().info('Capture service stub ready on /trigger_capture')

    def handle_capture(
        self,
        request: TriggerCapture.Request,
        response: TriggerCapture.Response,
    ) -> TriggerCapture.Response:
        self.get_logger().info(
            f'Received capture request with timeout_sec={request.timeout_sec}'
        )

        detection_1 = MarkerDetection()
        detection_1.marker_id = 42


        pose_1 = PoseStamped()
        pose_1.header.frame_id = 'base'
        pose_1.pose.position.x = 0.50
        pose_1.pose.position.y = 0.10
        pose_1.pose.position.z = 0.20
        pose_1.pose.orientation.w = 1.0
        detection_1.pose = pose_1

        detection_2 = MarkerDetection()
        detection_2.marker_id = 7
   

        pose_2 = PoseStamped()
        pose_2.header.frame_id = 'base'
        pose_2.pose.position.x = 0.62
        pose_2.pose.position.y = -0.08
        pose_2.pose.position.z = 0.19
        pose_2.pose.orientation.w = 1.0
        detection_2.pose = pose_2

        response.success = True
        response.detections = [detection_1, detection_2]
        response.message = 'Stub detections returned successfully.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CaptureServiceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()