## Pure Visual Pose Estimation Guide

This guide will walk you through the process of setting up and using the **Pure Visual Pose Estimation** system. The system allows you to estimate the pose of an object in the camera's field of view and provides the pose data with or without a display. Follow the steps below for installation, coordinate system calibration, and pose estimation methods.

### 1. Coordinate System Calibration

To start with coordinate system calibration, follow these steps:

1. Run the calibration application:
    ```bash
    python app.py
    ```

2. A window will open showing the camera feed. In this feed, place the calibration board on a flat surface (such as a table). The board's plane should align with the **xoy** plane, with the **z-axis** pointing upwards.

3. Press the **C** key to calibrate the coordinate system. After pressing **C**, the system will display the coordinate axes on the screen. You can adjust the position of the calibration board as needed to establish a suitable coordinate system.

4. Once the coordinate system is set, the system is ready for pose estimation.

### 2. Pose Estimation with Display

For a visual representation of the object's position within the camera's field of view, use the following steps:

1. Run the pose estimation application:
    ```bash
    python app.py
    ```

2. Place the object you want to track within the camera's field of view. The object will be marked in the display window, and its coordinates will be shown in the top-left corner of the window.

This mode provides real-time feedback both visually and numerically, which is useful for demonstrations or debugging.

### 3. Pose Estimation without Display

If you prefer to receive pose data without a visual interface, you can use the WebSocket-based system. Follow the steps below:

1. First, start the WebSocket server, which runs on port **8765** by default:
    ```bash
    python websocket_app.py
    ```

2. Then, run the WebSocket client to receive pose data:
    ```bash
    python client.py
    ```

3. The **client.py** script will connect to the WebSocket server and output the real-time pose data of the object. You can then process this data as needed.

This mode is ideal for headless environments or cases where only the raw data is required for further analysis or integration with other systems.
