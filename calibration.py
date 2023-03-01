#coding:utf-8
import cv2
import numpy as np
import glob
from tqdm import tqdm
import os

def camera_calibration(path,checkerboard_size,camera_name):

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    #棋盘格模板规格
    w = checkerboard_size[0]
    h = checkerboard_size[1]
    # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
    objp = np.zeros((w*h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    # 储存棋盘格角点的世界坐标和图像坐标对
    objpoints = []  # 在世界坐标系中的三维点
    imgpoints = []  # 在图像平面的二维点

    images = glob.glob(path)
    print(images)
    # img = cv2.imread(images[0])
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    for fname in tqdm(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # 找到棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, (w, h), None)

        # 如果找到足够点对，将其存储起来
        if ret is True:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            # 将角点在图像上显示
            cv2.drawChessboardCorners(img, (w, h), corners, ret)
            # cv2.imshow('findCorners', img)
            # cv2.waitKey(0)
    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not os.path.exists('Intrinsics/' + camera_name):
        os.makedirs('Intrinsics/' + camera_name)
    Intrinsics_path = 'Intrinsics/' + camera_name + '/'

    # print("ret:", ret)
    # print("mtx:\n", mtx)  # 内参数矩阵
    np.save(Intrinsics_path+'mtx.npy', mtx)
    # print("dist:\n", dist)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
    np.save(Intrinsics_path+'dist.npy', dist)
    # print("rvecs:\n", rvecs)  # 旋转向量  # 外参数
    np.save(Intrinsics_path+'rvecs.npy',rvecs)
    # print("tvecs:\n", tvecs)  # 平移向量  # 外参数
    np.save(Intrinsics_path+'tvecs.npy', tvecs)
    return ret, mtx, dist, rvecs, tvecs

if __name__ == '__main__':
    camera_calibration('calibration_image/sonya6000_chessboard/*.jpg',[11,8],'sony_a6000')
