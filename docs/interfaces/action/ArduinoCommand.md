[back](../percussionModule.md)
# ArduinoCommand action

## Overview

Action type used by the task manager to send generic commands to the Arduino over serial. The node opens a serial connection per goal, sends a pipe-delimited message, and streams all Arduino responses as feedback until the Arduino replies with a terminal state (`DONE` or `ERROR`).

The `msg_type` field identifies the command and is used to match Arduino responses. The `data` field carries the command payload, and `msg_info` is optional metadata for logging or context.

## Content

```
# Goal
string msg_type
string data
string msg_info
---
# Result
bool success
string message
---
# Feedback
string state
```

