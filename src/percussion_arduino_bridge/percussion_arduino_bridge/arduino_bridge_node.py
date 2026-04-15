#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import serial

from percussion_interfaces.action import ArduinoCommand


class ArduinoBridgeNode(Node):
    """ROS 2 node: generic Arduino command action server over serial."""

    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout_sec', 10.0)

        self.port     = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout  = self.get_parameter('timeout_sec').value

        self._action_server = ActionServer(
            self,
            ArduinoCommand,
            'arduino_command',
            execute_callback=self._execute_command,
        )
        self.get_logger().info('ArduinoCommand action server started')

    async def _execute_command(self, goal_handle) -> ArduinoCommand.Result:
        msg_type = goal_handle.request.msg_type
        data     = goal_handle.request.data
        msg_info = goal_handle.request.msg_info

        result = ArduinoCommand.Result()

        try:
            ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1.0)
            time.sleep(0.5)  # wait for Arduino reset

            message = f'{msg_type}|{data}|{msg_info}\n'
            ser.write(message.encode('utf-8'))
            self.get_logger().debug(f'Sent: {message.strip()}')

            start = time.time()
            while (time.time() - start) < self.timeout:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if not line:
                        continue

                    self.get_logger().debug(f'Received: {line}')

                    # Forward raw line as feedback
                    fb = ArduinoCommand.Feedback()
                    fb.state = line
                    goal_handle.publish_feedback(fb)

                    # Parse MSG_TYPE|STATE|...
                    parts = line.split('|')
                    if len(parts) >= 2 and parts[0] == msg_type:
                        state = parts[1]
                        self.get_logger().info(f'Arduino: {msg_type} - {state}')
                        if state == 'DONE':
                            result.success = True
                            result.message = 'Command completed'
                            goal_handle.succeed()
                            break
                        if state == 'ERROR':
                            result.success = False
                            result.message = line
                            goal_handle.abort()
                            break
                else:
                    time.sleep(0.01)
            else:
                # Timeout
                result.success = False
                result.message = f'Timed out after {self.timeout}s'
                goal_handle.abort()

            return result

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()


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
