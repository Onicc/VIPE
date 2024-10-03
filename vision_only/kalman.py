
import numpy as np

class KalmanFilter:
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_z=0.0):
        self.x = np.array([initial_x, 0.0, initial_y, 0.0, initial_z, 0.0])  # 初始状态 [x, vx, y, vy, z, vz]
        self.P = np.eye(6)  # 初始状态协方差矩阵

        self.dt = 1/30.0  # 时间步长
        self.F = np.array([[1, self.dt, 0,  0,  0,  0],
                           [0,  1,  0,  0,  0,  0],
                           [0,  0,  1, self.dt,  0,  0],
                           [0,  0,  0,  1,  0,  0],
                           [0,  0,  0,  0,  1, self.dt],
                           [0,  0,  0,  0,  0,  1]])

        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]])

        self.Q = np.eye(6) * 0.01  # 过程噪声
        self.R = np.eye(3) * 1     # 测量噪声

    def update(self, position_x, position_y, position_z):
        # 测量
        z = np.array([position_x, position_y, position_z])
        
        # 预测
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # 更新
        y = z - self.H @ self.x  # 计算创新
        S = self.H @ self.P @ self.H.T + self.R  # 计算创新协方差
        K = self.P @ self.H.T @ np.linalg.inv(S)  # 计算卡尔曼增益
        
        self.x = self.x + K @ y  # 更新状态
        self.P = (np.eye(6) - K @ self.H) @ self.P  # 更新协方差矩阵
        
        return self.x[0], self.x[2], self.x[4]
