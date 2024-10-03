import asyncio
import websockets
import time
import json
import cv2
from camera import Camera
from kalman import KalmanFilter
from tracker import MarkerTracker

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

    async def send_pose(self, websocket):
        """Send the pose data via WebSocket to the connected client."""
        while True:
            frame = self.camera.get_frame()
            pose = self.marker_tracker.get_pose(frame)
            if pose:  # If pose is valid, send it
                # Add timestamp to the pose data
                pose['timestamp'] = time.time()
                message = json.dumps(pose)
                await websocket.send(message)
                print(f"Sent pose: {message}")
            await asyncio.sleep(1 / FPS)  # Control loop to match the FPS

    async def handler(self, websocket, path):
        """Handle incoming WebSocket connections and send pose data."""
        await self.send_pose(websocket)

    async def run(self):
        # Start the WebSocket server
        async with websockets.serve(self.handler, "localhost", 8765):
            print("WebSocket server started at ws://localhost:8765")
            await asyncio.Future()  # Keep the server running

if __name__ == "__main__":
    tracker_app = TrackerApp()
    asyncio.run(tracker_app.run())