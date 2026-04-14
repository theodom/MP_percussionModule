#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import serial
import time
from typing import Optional

from percussion_interfaces.action import Hammer


class ArduinoBridgeNode(Node):
    """ROS 2 node for Arduino bridge with Hammer action server."""

    def __init__(self):
        super().__init__('arduino_bridge_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # Create action server
        self._action_server = ActionServer(
            self,
            Hammer,
            'hammer',
            execute_callback=self._execute_hammer
        )
        self.get_logger().info('Hammer action server started')

    def _send_and_wait(
        self,
        msg_type: str,
        data: str,
        msg_info: str,
        timeout_sec: float
    ) -> Optional[str]:
        """
        Send message to Arduino and wait for response.

        Format: MSG_TYPE|DATA|MSG_INFO

        Args:
            msg_type: Message type (e.g., 'HAMMER')
            data: Data payload (e.g., cycle count)
            msg_info: Info/metadata (e.g., 'CYCLE_1' or empty string)
            timeout_sec: Timeout in seconds

        Returns:
            Response from Arduino or None if timeout
        """
        message = f'{msg_type}|{data}|{msg_info}\n'

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            time.sleep(0.5)  # Wait for Arduino to be ready

            # Send message
            ser.write(message.encode('utf-8'))
            self.get_logger().debug(f'Sent: {message.strip()}')

            # Wait for response with timeout
            start_time = time.time()
            while (time.time() - start_time) < timeout_sec:
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    if response:
                        self.get_logger().debug(f'Received: {response}')
                        return response
                time.sleep(0.01)

            self.get_logger().warn(f'Timeout waiting for response after {timeout_sec}s')
            return None

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            return None
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()

    def _parse_response(self, response: str) -> Optional[tuple[str, str]]:
        """Parse Arduino response format: MSG_TYPE|STATE|MSG_INFO"""
        try:
            parts = response.split('|')
            if len(parts) >= 2:
                msg_type = parts[0]
                state = parts[1]
                return (msg_type, state)
        except Exception as e:
            self.get_logger().error(f'Failed to parse response: {e}')
        return None

    async def _execute_hammer(self, goal_handle) -> Hammer.Result:
        """Execute hammer action."""
        cycle_length = goal_handle.request.cycle_length

        self.get_logger().info(
            f'Executing hammer with cycle_length={cycle_length}'
        )

        result = Hammer.Result()

        # Calculate timeout: 5 second per cycle + 5 second buffer
        timeout = (cycle_length * 5.0) + 5.0

        # Send command and wait for response
        response = self._send_and_wait(
            msg_type='HAMMER_REQ',
            data=str(cycle_length),
            msg_info='',
            timeout_sec=timeout
        )

        if not response:
            result.success = False
            result.message = 'No response from Arduino'
            goal_handle.abort()
            return result

        # Parse response
        parsed = self._parse_response(response)
        if not parsed:
            result.success = False
            result.message = 'Failed to parse Arduino response'
            goal_handle.abort()
            return result

        msg_type, state = parsed

        # Update feedback
        feedback_msg = Hammer.Feedback()
        feedback_msg.state = state
        goal_handle.publish_feedback(feedback_msg)

        # Check result
        if state == 'DONE':
            result.success = True
            result.message = 'Hammer completed successfully'
            goal_handle.succeed()
            self.get_logger().info('Hammer action succeeded')
        elif state == 'ERROR':
            result.success = False
            result.message = f'Arduino error: {response}'
            goal_handle.abort()
            self.get_logger().error('Hammer action failed')
        else:
            result.success = False
            result.message = f'Unexpected state: {state}'
            goal_handle.abort()
            self.get_logger().error(f'Unexpected state: {state}')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
