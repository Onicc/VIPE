## Markers Calibration Guide

This guide explains how to calibrate the relative positions of ArUco markers using the provided Python scripts. The calibration process involves capturing multiple images of the markers from different angles and then using these images to compute the relative positions of the markers. Follow the steps below for a detailed walkthrough.

### Steps for Calibrating ArUco Markers

#### 1. Image Collection

The first step in the calibration process is collecting images of the ArUco markers from various angles. Follow these instructions:

1. Run the script to begin capturing images:
   ```bash
   python collect_images.py
   ```

2. A window will open, showing the camera feed. Use this feed to ensure that the ArUco markers are clearly visible within the frame.

3. Press the **S** key to save an image. You should capture at least **15 images** from different angles to ensure an accurate calibration. Make sure that each image includes visible ArUco markers.

4. After collecting a sufficient number of images, you can enter **Q** to close the window and proceed to the next step.

#### 2. Marker Calibration

Once you have collected the images, you can run the calibration script to compute the relative positions of the ArUco markers:

1. Run the calibration script:
   ```bash
   python calibrate_markers.py
   ```

2. The script will process the captured images and calculate the relative positions of the ArUco markers. This calculation may take a few moments, depending on the number of images and the complexity of the marker configuration.

3. Upon completion, the script will generate a file containing the calibrated positions of the markers. The output file will be located at:
   ```
   markers_calibration/params/calibrated_marker_positions.json
   ```

#### 3. Copy Calibration Data

To finalize the process, copy the generated calibration data file to the correct directory for further use in the system:

1. Copy the `calibrated_marker_positions.json` file to the `vision_only/params` directory by running the following command:
   ```bash
   cp markers_calibration/params/calibrated_marker_positions.json vision_only/params
   ```

This step ensures that the calibration data is stored in the appropriate location for future processing or integration with the pose estimation system.
