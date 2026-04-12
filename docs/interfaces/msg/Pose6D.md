# Pose6D message

## Goal

ROS as standard  not use a 6D vector for Poses, which is used for the ur10e as well as previously written code. Therefore a custom interface  was made to reduce the number of transformations needed in the code.

## Content

```
float64 x
float64 y
float64 z
float64 rx
float64 ry
float64 rz
```