# TriggerCapture service

## Overview

This service takes a timeout time in seconds as input. The service server [`capture_service_node`](../../Perception/capture_service_node.md) then detects aruco marker. It fills a list of [`MarkerDetection`](../msg/MarkerDetection.md) messages and returns them in the response, along with a Success boolean value, and the frame in which the `MarkerDetection[]` are represented.

> `frame` value is currently unused.

## Content

```
int32 timeout_sec
---
bool success
percussion_interfaces/MarkerDetection[] detections
string frame 
string message