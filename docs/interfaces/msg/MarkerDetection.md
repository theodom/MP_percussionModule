# MarkerDetection Message

## Overview

Simple message type to add extra information to a 6D marker pose, such as the marker ID. This may turn out to be unneccesary.

## Content

```
int32 marker_id
percussion_interfaces/Pose6D pose
```