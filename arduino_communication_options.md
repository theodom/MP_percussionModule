# Arduino Communication Options: Serial Bridge vs. micro-ROS

## Option 1: Serial + ROS Bridge Node

A dedicated ROS 2 node runs on the host machine, reads serial data from the Arduino over USB, and republishes it on the ROS network.

### Pros
- Simple Arduino-side code — pure C++, no ROS dependency or RTOS required
- Easy to debug: open a serial monitor and inspect raw data at any point
- Full control over the protocol — can tune for low latency or reliability as needed
- No transport layer failures; serial over USB is well-understood and robust
- Arduino firmware is decoupled from ROS — can be tested standalone
- Easier to iterate quickly: change the bridge node without reflashing the Arduino

### Cons
- Extra node to maintain (the bridge)
- Added latency: serial read → parse → publish adds a hop vs. direct ROS messaging
- Need to define and maintain a custom serial protocol (framing, checksums, etc.)
- USB cable required between host PC and Arduino at all times

---

## Option 2: micro-ROS

The Arduino publishes and subscribes directly to ROS 2 topics/services over a transport (USB serial, UDP, or WiFi), using the micro-ROS client library.

### Pros
- Arduino is a first-class ROS node — topics, services, and parameters work natively
- No bridge node to write or maintain
- Can use WiFi transport (the Nano ESP32 has built-in WiFi), removing the USB cable requirement
- Consistent with the rest of the ROS graph — `ros2 topic echo` works directly on Arduino data

### Cons
- Significantly more complex Arduino-side setup: micro-ROS agent, RTOS (FreeRTOS), memory constraints
- micro-ROS agent process must run on the host alongside your other nodes
- Harder to debug — no simple serial monitor; failures are less transparent
- WiFi transport introduces variable latency and potential packet loss — problematic for time-critical percussion control
- Larger firmware footprint; Nano ESP32 RAM may become a constraint
- Tighter coupling between firmware and ROS version; firmware updates require more care

---

## Recommendation for This Project

**Serial + ROS Bridge** is the simpler overall solution:

- Total overhead: one bridge node on the host, simple serial protocol on the Arduino
- micro-ROS requires: installing the micro-ROS library, a micro-ROS agent process running on the host, and a ROS-aware firmware on the Arduino — more moving parts for the same result
- Since timing is handled on the Arduino side and the USB cable is present regardless, neither approach has an advantage there
- The bridge node fits naturally as a new `percussion_tool` package alongside the existing nodes
