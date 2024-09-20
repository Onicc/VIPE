#%%
import pickle
import numpy as np

with open("./data/imu_pose.pkl", "rb") as file:
    imu_pose_data = pickle.load(file)
    
with open("./data/tip_pose.pkl", "rb") as file:
    tip_pose_data = pickle.load(file)

#%%

import plotly.graph_objects as go

# Extract position data
tip_position_x = [pose['position']['x'] for pose in tip_pose_data]
tip_position_y = [pose['position']['z'] for pose in tip_pose_data]
tip_position_z = [-pose['position']['y'] for pose in tip_pose_data]

# # Create 3D scatter plot
# fig = go.Figure()

# fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='markers+lines', 
#                            marker=dict(size=5, color=np.arange(len(x)), colorscale='Viridis')))

# # Set axis titles
# fig.update_layout(scene=dict(
#     xaxis_title="X Position",
#     yaxis_title="Y Position",
#     zaxis_title="Z Position",
#     aspectmode='data'),
#     title="3D Position Plot")

# # Show plot
# fig.show()

#%%

import numpy as np

# 初始化状态向量和协方差矩阵
x = np.array([tip_position_x[0], 0, tip_position_y[0], 0, tip_position_z[0], 0])  # 初始状态 [x, vx, y, vy, z, vz]
P = np.eye(6)  # 初始状态协方差矩阵

# 定义状态转移矩阵 F 和测量矩阵 H
dt = 1/30.0  # 时间步长
F = np.array([[1, dt, 0,  0,  0,  0],
              [0,  1,  0,  0,  0,  0],
              [0,  0,  1, dt,  0,  0],
              [0,  0,  0,  1,  0,  0],
              [0,  0,  0,  0,  1, dt],
              [0,  0,  0,  0,  0,  1]])

H = np.array([[1, 0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 0, 1, 0]])

# 定义过程噪声和测量噪声协方差
Q = np.eye(6) * 0.01  # 过程噪声
R = np.eye(3) * 1     # 测量噪声

filter_position_x = []
filter_position_y = []
filter_position_z = []

for z in zip(tip_position_x, tip_position_y, tip_position_z):
    # 预测
    x = F @ x
    P = F @ P @ F.T + Q
    
    # 更新
    y = z - H @ x  # 计算创新
    S = H @ P @ H.T + R  # 计算创新协方差
    K = P @ H.T @ np.linalg.inv(S)  # 计算卡尔曼增益
    
    x = x + K @ y  # 更新状态
    P = (np.eye(6) - K @ H) @ P  # 更新协方差矩阵
    
    # print(f"更新后的状态: {x[:3]}")
    filter_position_x.append(x[0])
    filter_position_y.append(x[2])
    filter_position_z.append(x[4])
    
#%%

# Create 3D scatter plot
fig = go.Figure()

fig.add_trace(go.Scatter3d(x=tip_position_x, y=tip_position_y, z=tip_position_z, mode='markers', 
                           marker=dict(size=5, color=np.arange(len(tip_position_x)), colorscale='Viridis')))
fig.add_trace(go.Scatter3d(x=filter_position_x, y=filter_position_y, z=filter_position_z, mode='markers', 
                           marker=dict(size=5, color=np.arange(len(tip_position_x)), colorscale='Viridis')))

# Set axis titles
fig.update_layout(scene=dict(
    xaxis_title="X Position",
    yaxis_title="Y Position",
    zaxis_title="Z Position",
    aspectmode='data'),
    title="3D Position Plot")

# Show plot
fig.show()