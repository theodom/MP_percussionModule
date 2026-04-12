# ExecutionMotion action

## Overview 

Action type used to move the ur10e robot arm using the RTDE library. A type of motion is given as input, which is resolved by the action server. 

The `marker_pose` parameter is only used for motion type MOVE_TO_MARKER. 

The `approach_offset` parameter gives a general indicator for where to move. an offset/buffer for when moving to a marker, a relative coordinate, or a direction/speed,... depending on `motion_type`.



## Content

```
# Goal
string motion_type
Pose6D marker_pose
float64[6] approach_offset
---
# Result
bool success
string message
Pose6D final_tcp_pose
---
# Feedback
string current_phase
```