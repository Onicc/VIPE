from camera import Camera
from kalman import KalmanFilter
from tracker import MarkerTracker
import cv2

FPS = 30
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

class TrackerApp:
    def __init__(self):
        self.camera = Camera(0, FRAME_WIDTH, FRAME_HEIGHT, FPS)
        self.marker_tracker = MarkerTracker(
            camera_params="params/camera_params.yml",
            markers_params="params/calibrated_marker_positions.json",
            transform_params="params/transform_params.json",
            width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=FPS)
    
    def run(self):
        while True:
            frame = self.camera.get_frame()
            
            key = cv2.waitKey(1)
            if key == ord("c"):
                self.marker_tracker.estimate_camera_pose_charuco(frame)
            if key == ord("q"):
                break
            
            pose = self.marker_tracker.get_pose(frame)
            print(pose)
            
            cv2.imshow("Tracker", frame)
                

if __name__ == "__main__":
    tracker_app = TrackerApp()
    tracker_app.run()