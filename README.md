# VIPE (Visual-Inertial Pose Estimation)

VIPE is a fork of the [DPOINT](https://github.com/Jcparkyn/dpoint) project, designed to make visual-inertial pose estimation easily applicable across various scenarios.

## Key Features

1. **Universal Bluetooth Support**: Leveraging the ArduinoBLE library, VIPE supports multiple boards, enabling more flexible hardware options.
2. **IMU Data Transmission**: IMU data can be transmitted via serial communication, offering an alternative to wireless solutions.
3. **Dual Version Support**: VIPE includes both pure visual tracking and combined visual-inertial pose estimation versions, providing users with flexibility based on their needs.
4. **Attachable to Any Object**: VIPE can be mounted on any object, enabling accurate pose estimation regardless of the application.
5. **ROS2 Integration**: Fully compatible with ROS2, VIPE can function as a ROS2 node, allowing seamless integration into robotic systems.

## Getting Started

### 1. Python Package Installation

To install all required Python packages, use the following command:

```bash
pip install -r requirements.txt
```

### 2. Camera Calibration

Before proceeding with pose estimation, camera calibration is required to ensure accuracy in measurements. We provide a tool to calibrate your camera.

For detailed instructions, please refer to the camera calibration README:

👉 [Camera Calibration Guide](camera_calibration/README.md)

### 3. Markers Calibration

After calibrating your camera, the next step is to calibrate the relative positions of the ArUco markers.

We provide a tool to assist you with this process. For detailed instructions on how to perform the markers calibration, please refer to the Markers Calibration README:

👉 [Markers Calibration Guide](markers_calibration/README.md)

### 4. Pure Visual Pose Estimation

If you are using pure visual methods (without IMU), the following section will guide you through the process of setting up and running the pure visual pose estimation pipeline.

For detailed instructions, please refer to the pure visual pose estimation README:

👉 [Pure Visual Pose Estimation Guide](vision_only/README.md)

### 5. Visual-Inertial Pose Estimation

For projects involving both vision and IMU data, the visual-inertial pose estimation is recommended. This setup provides more robust and accurate results, especially in challenging environments.

For detailed instructions, please refer to the visual-inertial pose estimation README:

👉 [Visual-Inertial Pose Estimation Guide](visual_inertial/README.md)

### 6. Running ROS2 Nodes

The ROS2 version of the visual-inertial pose estimation pipeline is available on the ros2 branch. Please switch to the ros2 branch by using the following command:

```bash
git checkout ros2
```
