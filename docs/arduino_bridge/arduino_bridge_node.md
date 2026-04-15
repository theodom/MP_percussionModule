# ArduinoBridgeNode

## Overview

This package is qutie simple. It receives an [`ArduinoCommand](../interfaces/action/ArduinoCommand.md) Action and opens serial communication with the arduino nano. It then continuously feeds back serial data to ros until it receives a state `DONE`.

## Components

- [`__init__`](#__init__): Node startup — declares serial parameters and creates the [`ArduinoCommand](../interfaces/action/ArduinoCommand.md) action server.
- [`_execute_command`](#_execute_command): Async action handler — opens serial, sends pipe-delimited command, polls for response, publishes feedback, returns result.

---

### `__init__`

**Declare Parameters**:

- `port`: Serial device path. Default: `/dev/ttyACM0`
- `baudrate`: Serial baud rate. Default: `115200`
- `timeout_sec`: Max wait time for Arduino response (seconds). Default: `10.0`

**Setup ROS interfaces**:

- `arduino_command`: `ActionServer` of type `ArduinoCommand` — the only ROS interface. All incoming goals are routed to `_execute_command`.

**Initialization**:

Logs `'ArduinoCommand action server started'` on successful startup.

---

### `_execute_command`

**Parameters**:

- `goal_handle`: ROS 2 action goal handle providing access to the request and methods `succeed()`, `abort()`, `publish_feedback()`.

**Return**:

- `ArduinoCommand.Result` — `success` (bool) and `message` (str).

**Goal request fields**:

- `msg_type`: string — command identifier (echoed back in Arduino response).
- `data`: string — command payload.
- `msg_info`: string — optional metadata.

**Serial protocol**:

**Node → Arduino**: `{msg_type}|{data}|{msg_info}\n` (UTF-8 encoded)

**Arduino → Node**: `{msg_type}|{STATE}|...\n` (pipe-delimited, UTF-8 decoded)

| Response format | Meaning | Action |
| --- | --- | --- |
| `{msg_type}\|DONE` | Command succeeded | `goal_handle.succeed()` → success result |
| `{msg_type}\|ERROR` | Command failed | `goal_handle.abort()` → failure result |
| `{msg_type}\|*` (other) | Intermediate feedback | Publish as feedback and continue polling |
| `(timeout)` | No DONE/ERROR within `timeout_sec` | `goal_handle.abort()` → timeout result |
| `SerialException` | Port unavailable, permission denied, etc. | `goal_handle.abort()` → error result |

**Execution flow**:

1. Opens serial connection on `port` with `baudrate` and read timeout `1.0s` (hard-coded, separate from node-level `timeout_sec`).
2. Sleeps `0.5s` to absorb Arduino hardware reset triggered by serial open.
3. Constructs message as `f'{msg_type}|{data}|{msg_info}\n'` and writes to serial.
4. **Polling loop** (wall-clock timeout `timeout_sec`):
   - If bytes available in receive buffer: reads one line, decodes UTF-8, strips whitespace.
   - Publishes every non-empty line as `Feedback.state` (raw serial text).
   - Parses line by splitting on `'|'`:
     - If `parts[0] == msg_type` and `parts[1] == 'DONE'`: sets `success=True`, calls `succeed()`, breaks.
     - If `parts[0] == msg_type` and `parts[1] == 'ERROR'`: sets `success=False` with message = full line, calls `abort()`, breaks.
   - If no bytes: sleeps `0.01s` (avoid busy-waiting).
5. **Timeout** (loop exits normally after `timeout_sec`): sets `success=False`, message = `'Timed out after Ns'`, calls `abort()`.
6. **Exception** (`SerialException`): sets `success=False`, message = exception string, calls `abort()`.
7. **Finally**: closes serial port.

---


## Design Notes

- **Per-goal serial connection**: The port is opened fresh for every action goal and closed after, which means the Arduino experiences a hardware reset (via the DTR line) on each new goal. The `0.5s` sleep absorbs the reset.

> This is handy for testing and for avoiding error on disconnecting arduino when it's not being used. But suboptimal for process time. 

- **Hard-coded serial read timeout**: `serial.Serial(timeout=1.0)` is distinct from the node-level `timeout_sec` parameter. The `1.0s` governs how long a single `readline()` blocks; the node-level `timeout_sec` governs the outer polling loop.
- **Feedback on every line**: All non-empty lines from the Arduino are immediately published as feedback before terminal-state parsing, so action clients observe intermediate progress messages.
- **Response filtering**: Lines that do not begin with the sent `msg_type` are silently ignored (still published as feedback). This tolerates Arduino debug prints or concurrent output.

