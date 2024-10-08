## Markers Calibration

To calibrate the relative positions of the ArUco markers, follow these steps:

1. Run the script to collect images:
   ```bash
   python collect_images.py
   ```

2. Press `s` to save an image. Capture at least 15 images from different angles, ensuring that each image includes visible ArUco markers.

3. Once you have collected the images, run the calibration script:
   ```bash
   python calibrate_markers.py
   ```

4. The script will generate the relative positions of the markers. The output file will be located at:
   ```
   markers_calibration/params/calibrated_marker_positions.json
   ```

5. Copy the generated `calibrated_marker_positions.json` file to the `vision_only/params`
   ```
   cp markers_calibration/params/calibrated_marker_positions.json vision_only/params
   ```

By following these steps, you will successfully calibrate the ArUco markers and store the calibration data in the appropriate location for further processing.