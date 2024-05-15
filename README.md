# RS2_Group_18 - Localisation Branch

This branch is used for detecting ArUco markers (using ChAruco Board for precise accuracy) for pose estimation of the robot arm.

## Requisites

Before using this code, make sure you have the following installed:

- [ROS2](https://docs.ros.org/en/rolling/Releases.html) (Humble/Iron/Rolling/etc.)
  Refer to the link above to access and install the correct distribution.

- Install the `realsense2` package for ROS2 to use the depth camera D435i. Instructions can be found [here](https://github.com/IntelRealSense/realsense-ros).

## Aruco Markers (Board Generation)

This can be used to generate the markers to implement the pose estimation detection, with the IDs, Board Width (mm), Board Height (mm), Dictionary, etc.

-For board or markers/ ChAruco (multiple markers), can be generated [here](https://calib.io/pages/camera-calibration-pattern-generator).

