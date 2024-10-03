import cv2
import numpy as np

class Camera:
    def __init__(self, camera_id, width, height, fps):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.focus_targets = np.array(
            [
                [0.1, 75],
                [0.15, 50],
                [0.2, 40],
                [0.3, 30],
                [0.5, 25],
            ]
        )

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to read frame from camera")
        return frame

    def release(self):
        self.cap.release()
        
    def get_focus_target(self, dist_to_camera):
        f = np.interp([dist_to_camera], self.focus_targets[:, 0], self.focus_targets[:, 1])[0]
        return 5 * round(f / 5)  # Webcam only supports multiples of 5
    
    def autofocus(self, dist_to_camera):
        focus = self.get_focus_target(dist_to_camera)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, focus)