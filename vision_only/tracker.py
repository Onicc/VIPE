import cv2
from cv2 import aruco
import numpy as np
import math
import json
import time
from typing import Optional, Tuple
from kalman import KalmanFilter

TARFET_OFFSET = (0.0, 0.06, 0.0)
TEXT_COL = (0, 0, 255)

MarkerDict = dict[int, tuple[np.ndarray, np.ndarray]]

class ArucoHelper:
    @staticmethod
    def get_aruco_params():
        p = aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        p.cornerRefinementWinSize = 2
        # Reduce the number of threshold steps, which significantly improves performance
        p.adaptiveThreshWinSizeMin = 15
        p.adaptiveThreshWinSizeMax = 15
        p.useAruco3Detection = False
        p.minMarkerPerimeterRate = 0.02
        p.maxMarkerPerimeterRate = 2
        p.minSideLengthCanonicalImg = 16
        p.adaptiveThreshConstant = 7
        return p
    
    @staticmethod
    def inverse_RT(rvec, tvec) -> Tuple[np.ndarray, np.ndarray]:
        R, _ = cv2.Rodrigues(rvec)
        Rt = np.transpose(R)
        return (cv2.Rodrigues(Rt)[0], -Rt @ tvec)

    @staticmethod
    def relative_transform(rvec1, tvec1, rvec2, tvec2) -> Tuple[np.ndarray, np.ndarray]:
        rvec2inv, tvec2inv = ArucoHelper.inverse_RT(rvec2, tvec2)
        rvec, tvec, *_ = cv2.composeRT(rvec1, tvec1, rvec2inv, tvec2inv)
        return (rvec, tvec)

class MarkerTracker:
    def __init__(
        self,
        camera_params: str,
        markers_params: str,
        transform_params: str,
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
    ):
        self.transform_params_path = transform_params
        self.width = width
        self.height = height
        self.fps = fps
        
        charuco_dic = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        charuco_params = aruco.DetectorParameters()

        self.charuco_board = aruco.CharucoBoard((12, 8), 0.024, 0.018, charuco_dic)
        self.charuco_board.setLegacyPattern(True)
        self.charuco_detector = aruco.ArucoDetector(charuco_dic, charuco_params)
        self.reprojection_error_threshold = 3
        
        self.cameraMatrix, self.distCoeffs = self.read_camera_parameters(camera_params)
        self.markerPositions  = self.load_marker_positions(markers_params)
        self.rvec: Optional[np.ndarray] = None
        self.tvec: Optional[np.ndarray] = None
        self.initialized = False
        self.allObjectPoints = np.concatenate(list(self.markerPositions.values()))
        self.lastValidMarkers: MarkerDict = {}
        self.lastVelocity = np.zeros(2)
        
        self.kalman_filter = KalmanFilter()
        
        self.load_transform_params()
        
    def load_transform_params(self):
        try:
            with open(self.transform_params_path, "r") as f:
                params = json.load(f)
                self.baseRvec = np.array(params["rvec"])
                self.baseTvec = np.array(params["tvec"])
        except:
            self.baseRvec = np.zeros([3, 1])
            self.baseTvec = np.zeros([3, 1])
            # raise Exception("Couldn't open transform params file, please calibrate first.")
            print("Couldn't open transform params file, please calibrate first.")

    def estimate_camera_pose_charuco(self, frame):
        camera_matrix = self.cameraMatrix
        dist_coeffs = self.distCoeffs
        corners, ids, rejected = self.charuco_detector.detectMarkers(frame)
        if len(corners) == 0:
            raise Exception("No markers detected")
        display_frame = aruco.drawDetectedMarkers(image=frame, corners=corners)
        num_corners, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners, markerIds=ids, image=frame, board=self.charuco_board
        )
        if num_corners < 5:
            raise Exception("Not enough corners detected")
        display_frame = aruco.drawDetectedCornersCharuco(
            image=display_frame, charucoCorners=charuco_corners, charucoIds=charuco_ids
        )
        success, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.charuco_board,
            camera_matrix,
            dist_coeffs,
            None,
            None,
            False,
        )
        if not success:
            raise Exception("Failed to estimate camera pose")
        # The rvec from charuco is z-down for some reason.
        # This is a hack to convert back to z-up.
        rvec, *_ = cv2.composeRT(np.array([0, 0, -np.pi / 2]), tvec * 0, rvec, tvec)
        rvec, *_ = cv2.composeRT(np.array([0, np.pi, 0]), tvec * 0, rvec, tvec)
        display_frame = cv2.drawFrameAxes(
            display_frame, camera_matrix, dist_coeffs, rvec, tvec, 0.2
        )
        # cv2.imshow("Charuco", display_frame)
        # return (rvec, tvec)
        self.baseRvec = rvec
        self.baseTvec = tvec
        
        with open(self.transform_params_path, "w") as f:
            json.dump(
                {
                    "rvec": rvec.tolist(),
                    "tvec": tvec.tolist(),
                },
                f,
            )

    def read_camera_parameters(self, filename: str) -> Tuple[np.ndarray, np.ndarray]:
        fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise Exception("Couldn't open file")
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        return (camera_matrix, dist_coeffs)
    
    def load_marker_positions(self, filename: str) -> MarkerDict:
        try:
            with open(filename, "r") as f:
                pos_json = json.load(f)
                return {int(k): np.array(v) for k, v in pos_json.items()}
        except:
            raise Exception("Couldn't open marker positions file, please calibrate first.")
    
    def array_to_str(self, arr):
        return ",".join(map(lambda x: f"{x:+2.2f}", list(arr.flat)))

    def vector_rms(self, arr: np.ndarray, axis: int):
        """Computes the RMS magnitude of an array of vectors."""
        return math.sqrt(np.mean(np.sum(np.square(arr), axis=axis)))

    def bounds(self, x):
        return np.min(x), np.max(x)

    def clamp(self, x, xmin, xmax):
        return max(min(x, xmax), xmin)

    def solve_pnp(
        self,
        initialized,
        prev_rvec,
        prev_tvec,
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        """Attempt to refine the previous pose. If this fails, fall back to SQPnP."""
        if initialized:
            rvec, tvec = cv2.solvePnPRefineVVS(
                object_points,
                image_points,
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs,
                # OpenCV mutates these arguments, which we don't want.
                rvec=prev_rvec.copy(),
                tvec=prev_tvec.copy(),
            )
            projected_image_points, _ = cv2.projectPoints(
                object_points, rvec, tvec, camera_matrix, dist_coeffs, None
            )
            projected_image_points = projected_image_points[:, 0, :]
            reprojection_error = self.vector_rms(projected_image_points - image_points, axis=1)

            if reprojection_error < self.reprojection_error_threshold:
                return (True, rvec, tvec)
            else:
                print(f"Reprojection error too high: {reprojection_error}")

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_SQPNP,
        )
        return (success, rvec, tvec)

    def detect_markers_bounded(self, frame: np.ndarray, x0: int, x1: int, y0: int, y1: int):
        x0, y0 = max(x0, 0), max(y0, 0)
        frame_view = frame[y0:y1, x0:x1]
        ids = None
        allCornersIS = []
        rejected = []
        try:
            allCornersIS, ids, rejected = self.charuco_detector.detectMarkers(frame_view)
        except cv2.error as e:
            # OpenCV threw an error here once for some reason, but we'd rather ignore it.
            # D:\a\opencv-python\opencv-python\opencv\modules\objdetect\src\aruco\aruco_detector.cpp:698: error: (-215:Assertion failed) nContours.size() >= 2 in function 'cv::aruco::_interpolate2Dline'
            print(e)
            pass
        if ids is not None:
            for i in range(ids.shape[0]):
                allCornersIS[i][0, :, 0] += x0
                allCornersIS[i][0, :, 1] += y0
        return allCornersIS, ids, rejected
    

    def get_search_area(self, rvec: np.ndarray, tvec: np.ndarray, velocity: np.array):
        """Returns a bounding box to search in the next frame, based on the current marker positions and velocity."""

        # Re-project all object points, to avoid cases where some markers were missed in the previous frame.
        projected_image_points, _ = cv2.projectPoints(
            self.allObjectPoints, rvec, tvec, self.cameraMatrix, self.distCoeffs, None
        )
        projected_image_points = projected_image_points[:, 0, :]

        x0, x1 = self.bounds(projected_image_points[:, 0] + velocity[0] / self.fps)
        y0, y1 = self.bounds(projected_image_points[:, 1] + velocity[1] / self.fps)
        w = x1 - x0
        h = y1 - y0

        # Amount to expand each axis by, in pixels. This is just a rough heuristic, and the constants are arbitrary.
        expand = max(0.5 * (w + h), 200) + 1.0 * np.abs(velocity) / self.fps

        # Values are sometimes extremely large if tvec is wrong, clamp is a workaround to stop cv2.rectangle from breaking.
        return (
            int(self.clamp(x0 - expand[0], 0, self.width)),
            int(self.clamp(x1 + expand[0], 0, self.width)),
            int(self.clamp(y0 - expand[1], 0, self.height)),
            int(self.clamp(y1 + expand[1], 0, self.height)),
        )

    def process_frame(self, frame: np.ndarray):
        ids: np.ndarray
        if self.initialized:
            x0, x1, y0, y1 = self.get_search_area(
                self.rvec, self.tvec, self.lastVelocity
            )
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 100, 0, 0.3), 2)
            allCornersIS, ids, rejected = self.detect_markers_bounded(frame, x0, x1, y0, y1)
        else:
            allCornersIS, ids, rejected = self.charuco_detector.detectMarkers(frame)
        aruco.drawDetectedMarkers(frame, allCornersIS, ids)
        valid_markers: MarkerDict = {}
        if ids is not None:
            for i in range(ids.shape[0]):
                # cornersIS is 4x2
                id, cornersIS = (ids[i, 0], allCornersIS[i][0, :, :])
                if id in self.markerPositions:
                    cornersPS = self.markerPositions[id]
                    valid_markers[id] = (cornersPS, cornersIS)

        if len(valid_markers) < 1:
            self.initialized = False
            self.lastValidMarkers = {}
            self.next_search_area = None
            return None

        point_deltas = []
        for id, (cornersPS, cornersIS) in valid_markers.items():
            if id in self.lastValidMarkers:
                velocity = cornersIS - self.lastValidMarkers[id][1]
                point_deltas.append(np.mean(velocity, axis=0))

        if point_deltas:
            meanVelocity = np.mean(point_deltas, axis=0) * 30  # px/second
        else:
            meanVelocity = np.zeros(2)

        mean_position_IS = np.mean(
            [cornersIS for _, cornersIS in valid_markers.values()],
            axis=(0, 1),
        )

        screen_corners = []
        pen_corners = []
        delay_per_image_row = 1 / 30 / 1080  # seconds/row

        for id, (cornersPS, cornersIS) in valid_markers.items():
            pen_corners.append(cornersPS)
            if point_deltas:
                # Compensate for rolling shutter
                timeDelay = (
                    cornersIS[:, 1] - mean_position_IS[1]
                ) * delay_per_image_row  # seconds, relative to centroid
                cornersISCompensated = (
                    cornersIS - meanVelocity * timeDelay[:, np.newaxis]
                )
                screen_corners.append(cornersISCompensated)
            else:
                screen_corners.append(cornersIS)

        self.initialized, self.rvec, self.tvec = self.solve_pnp(
            self.initialized,
            self.rvec,
            self.tvec,
            object_points=np.concatenate(pen_corners),
            image_points=np.concatenate(screen_corners),
            camera_matrix=self.cameraMatrix,
            dist_coeffs=self.distCoeffs,
        )

        self.lastValidMarkers = valid_markers
        self.lastVelocity = meanVelocity
        return (self.rvec, self.tvec)
    
    def get_pose(self, frame: np.ndarray):
        result = self.process_frame(frame)
        if result is not None:
            rvec, tvec = result
            offset = -np.array(TARFET_OFFSET)
            rvec, tvec, *_ = cv2.composeRT(
                np.zeros(3), offset, rvec, tvec
            )
            rvec_relative, tvec_relative = ArucoHelper.relative_transform(
                rvec, tvec, self.baseRvec, self.baseTvec
            )
            
            r_relative = cv2.Rodrigues(rvec_relative)[0]
            filter_x, filter_y, filter_z = self.kalman_filter.update(tvec_relative[0][0], tvec_relative[1][0], tvec_relative[2][0])
            
            pose = {
                "timestamp": time.time(),
                "position": {
                    "x" : filter_x,
                    "y" : filter_y,
                    "z" : filter_z
                },
                "orientation": {
                    "x" : r_relative[0][0],
                    "y" : r_relative[1][0],
                    "z" : r_relative[2][0],
                    "w" : r_relative[0][1]
                }
            }
            
            cv2.drawFrameAxes(frame, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.01)
            cv2.putText(
                frame,
                "Position: [{:.2f},{:.2f},{:.2f}]cm".format(pose['position']['x'], pose['position']['y'], pose['position']['z']),
                (10, 120),
                cv2.FONT_HERSHEY_DUPLEX,
                1,
                TEXT_COL,
            )
                            
            return pose

        return None