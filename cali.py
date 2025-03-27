import numpy as np

# 输入四点坐标
points = np.array([
    [325.974, 117.393, 980],
    [382.402, 113.233, 947],
    [371.401, 173.756, 920],
    [308.505, 179.312, 946]
])

# 构造矩阵 A 和向量 B
A = np.hstack([points[:, :2], np.ones((4, 1))])
B = points[:, 2]

# 最小二乘法拟合平面
a, b, d = np.linalg.lstsq(A, B, rcond=None)[0]
c = -1

# 计算点到平面的距离
distances = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)

# 计算平面度
flatness = np.max(distances)
print("平面度:", flatness)