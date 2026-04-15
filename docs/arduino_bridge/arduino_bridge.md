[back](../percussionModule.md#arduino-bridge)
# Arduino Bridge Package

## Overview

ROS 2 package providing a generic serial communication interface to the Arduino via a single `ArduinoCommand` action server. The node opens a serial connection per action goal, sends pipe-delimited commands, and forwards all Arduino responses as action feedback.

## Components

- Node: [`arduino_bridge_node`](./arduino_bridge_node.md)

## Goals

The Arduino bridge decouples the task manager from Arduino-specific communication logic by providing a unified action-based interface.

- Expose a generic `ArduinoCommand` action server for commanding the Arduino
- Send pipe-delimited messages (`msg_type|data|msg_info\n`) over serial
- Forward all Arduino responses as action feedback in real time
- Distinguish terminal states: DONE (success), ERROR (abort), or timeout (abort)
- Handle serial exceptions gracefully (SerialException → abort)

Makes use of: PySerial
