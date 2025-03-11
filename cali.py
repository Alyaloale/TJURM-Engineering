import cv2
import numpy as np
import glob

# 棋盘格内角点数量（宽×高）
CHECKERBOARD = (12, 9)
# 单个棋盘格实际边长（单位：毫米）
SQUARE_SIZE_MM = 15.0

# 终止条件，用于亚像素级角点检测
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 生成理论 3D 点（带物理单位）
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE_MM

# 存储 3D 点（物理单位，毫米）和 2D 点（像素坐标）
objpoints = []
imgpoints = []

# 获取所有标定图像的文件名
images = glob.glob('/home/tjurm/Code/TJURM-Engineering/image/biaoding/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"警告：无法读取图像 {fname}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格内角点
    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, None)

    if ret:
        print(f"成功在 {fname} 中检测到角点")
        objpoints.append(objp)

        # 亚像素级精确化角点位置
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 绘制并显示角点
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
    else:
        print(f"未在 {fname} 中检测到角点")

cv2.destroyAllWindows()

# 检查 objpoints 和 imgpoints 的长度
print(f"objpoints 长度: {len(objpoints)}")
print(f"imgpoints 长度: {len(imgpoints)}")

if len(objpoints) > 0 and len(imgpoints) > 0:
    # 进行相机标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        print("相机标定成功！")
        print("相机内参矩阵:")
        print(mtx)
        print("畸变系数:")
        print(dist)

        # 计算重投影误差
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        print("重投影误差的平均值: {}".format(mean_error / len(objpoints)))

        # 保存标定结果
        np.savez('camera_calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
        print("标定结果已保存到 camera_calibration.npz")
    else:
        print("相机标定失败！")
else:
    print("没有有效的角点检测结果，无法进行相机标定。")
