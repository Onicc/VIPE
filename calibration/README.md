## Camera Calibration Using Charuco Board

### Overview

This section contains two Python scripts for camera calibration using a Charuco board:

1. **`collect_images.py`**: A script for capturing images of a Charuco board from a camera.
2. **`calibrate_camera.py`**: A script for calibrating the camera using the captured Charuco board images.

---

### Prerequisites

Before running the scripts, ensure you have the following installed:

- Python 3.x
- OpenCV (`opencv-python` and `opencv-contrib-python`)
- Numpy

To install the necessary libraries, run the following command:
```bash
pip install numpy opencv-python opencv-contrib-python
```

Additionally, download the Charuco board PDF file, `calib.io_charuco_297x210_8x12_24_18_DICT_4X4.pdf`, and print it. You will need to capture images of this printed Charuco board using your camera.

---

### Charuco Board Details

The Charuco board used in this project is configured as follows:

- **Dimensions**: 297mm x 210mm (A4 paper size)
- **Number of Rows**: 8
- **Number of Columns**: 12
- **Square Size**: 24mm
- **Marker Size**: 18mm
- **Aruco Dictionary**: DICT_4X4_100

---

### Step 1: Collect Images Using `collect_images.py`

First, capture several images of the Charuco board from different angles to use for calibration.

1. **Prepare the Workspace:**
   - Print the Charuco board (`calib.io_charuco_297x210_8x12_24_18_DICT_4X4.pdf`).
   - Ensure good lighting and position the board in different angles within the camera's view to get a variety of images for calibration.

2. **Run the Image Collection Script:**

   ```bash
   python collect_images.py
   ```

   This script will open the camera and display the video feed. The following controls are available:
   
   - Press **`s`** to save an image of the current frame.
   - Press **`q`** to quit the image collection process.

3. **Save the Images:**
   - Images are saved in the `./images` folder with filenames in the format `image_YYYYMMDD_HHMMSS.png`.
   - Ensure that the `./images` folder exists. If not, create it manually before running the script.

---

### Step 2: Calibrate the Camera Using `calibrate_camera.py`

Once you have collected a sufficient number of images (at least 10), proceed with camera calibration.

1. **Ensure the Image Folder Exists:**

   The `calibrate_camera.py` script looks for images in the `./images` folder. Ensure that this folder contains the images captured in the previous step.

2. **Run the Calibration Script:**

   ```bash
   python calibrate_camera.py
   ```

   The script performs the following operations:
   
   - Reads all PNG images from the `./images` folder.
   - Detects the Aruco markers and Charuco corners in each image.
   - Performs the camera calibration using the detected points.
   - Outputs the camera matrix and distortion coefficients.
   - Saves the calibration parameters to a YAML file (`camera_params.yml`) in the `./params` folder.

   During execution, the script will display each image with the detected Charuco markers and corners. Press any key to proceed to the next image.

---

### Output

After running the calibration script, you will get the following outputs:

- **Camera Matrix (`camera_matrix`)**: Contains the intrinsic camera parameters, including focal length and optical center.
- **Distortion Coefficients (`distortion_coefficients`)**: Describes lens distortion parameters.

The results are saved to `./params/camera_params.yml`. Example content of this file might look like:

```yaml
%YAML:1.0
---
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ k1, k2, p1, p2, k3 ]
```

Where `fx`, `fy` are the focal lengths, `cx`, `cy` are the optical center coordinates, and `k1, k2, p1, p2, k3` are distortion coefficients.

---

### Common Issues

- **No Charuco Board Detected**: If the script fails to detect a Charuco board, ensure that the board is well-lit, the images are clear, and the printed board has sharp contrast between the white squares and black markers.
- **Not Enough Detected Points**: If the detection response is low (less than 20 points), ensure that the Charuco board is fully visible in the image and that there is no obstruction or blurring.
- **Camera Access**: If the camera does not open, ensure that it is correctly connected and that the correct camera ID is used in `cv2.VideoCapture(0)`.
