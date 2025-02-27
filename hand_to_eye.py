# 文件说明：棋盘格手眼标定（眼在手外）

'''
在执行手眼标定时，需要将标定板固定在机械臂末端，并同时将相机固定在另一侧。
接着控制机械臂末端位于不同的位置，记录下此时机械臂相对于基座的位姿，并使用相机拍摄标定板上的棋盘格图像。
将图像放入./images文件夹中，并将位姿信息输入到chessboard_handeye_calibration.py文件的pose_vectors变量中。
最后运行chessboard_handeye_calibration.py，即可得到相机相对于机械臂基座的位姿矩阵。
'''

import cv2
import numpy as np
import transforms3d
import glob
import re
import os
import argparse

def pose_vectors_to_base2end_transforms(pose_vectors):
    # 提取旋转矩阵和平移向量
    R_base2ends = []
    t_base2ends = []

    # 迭代的到每个位姿的旋转矩阵和平移向量
    for pose_vector in pose_vectors:
        # 提取旋转矩阵和平移向量
        R_end2base = euler_to_rotation_matrix(pose_vector[3], pose_vector[4], pose_vector[5])
        t_end2base = pose_vector[:3]

        # 将旋转矩阵和平移向量组合成齐次位姿矩阵
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = R_end2base
        pose_matrix[:3, 3] = t_end2base

        # 求位姿矩阵的逆矩阵
        pose_matrix_inv = np.linalg.inv(pose_matrix)

        # 提取旋转矩阵和平移向量
        R_base2end = pose_matrix_inv[:3, :3]
        t_base2end = pose_matrix_inv[:3, 3]

        # 将平移向量转换为列向量(3*1)
        t_base2end = t_base2end.reshape(3, 1)

        # 将旋转矩阵和平移向量保存到列表
        R_base2ends.append(R_base2end)
        t_base2ends.append(t_base2end)

    return R_base2ends, t_base2ends

def euler_to_rotation_matrix(rx, ry, rz, unit='deg'):
    '''
    将欧拉角转换为旋转矩阵：R = Rz * Ry * Rx
    :param rx: x轴旋转角度
    :param ry: y轴旋转角度
    :param rz: z轴旋转角度
    :param unit: 角度单位，'deg'表示角度，'rad'表示弧度
    :return: 旋转矩阵
    '''
    if unit =='deg':
        # 把角度转换为弧度
        rx = np.radians(rx)
        ry = np.radians(ry)
        rz = np.radians(rz)

    # 计算旋转矩阵Rz 、 Ry 、 Rx
    Rx = transforms3d.axangles.axangle2mat([1, 0, 0], rx)
    Ry = transforms3d.axangles.axangle2mat([0, 1, 0], ry)
    Rz = transforms3d.axangles.axangle2mat([0, 0, 1], rz)

    # 计算旋转矩阵R = Rz * Ry * Rx
    rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))

    return rotation_matrix

def main(pose_file, image_folder):
    # 输入位姿数据
    pose_vectors = np.loadtxt(pose_file, delimiter=',')
    print("Loaded pose vectors from CSV:")
    print(pose_vectors)

    # 定义棋盘格参数
    square_size = 50  # 假设格子的边长为30mm
    pattern_size = (4, 3)   # 在这个例子中，假设标定板有9个内角点和6个内角点

    # 导入相机内参和畸变参数
    camera_matrix = np.array([[1123.9, 0, 982.364],
                              [0, 1123.4, 567.264],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.0769521, -0.105434, 6.25417e-05, 3.9459e-5,0.0428337], dtype=np.float32)
    K = camera_matrix

    # 准备位姿数据
    obj_points = []  # 用于保存世界坐标系中的三维点
    img_points = []  # 用于保存图像平面上的二维点

    # 创建棋盘格3D坐标
    objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    # 迭代处理图像
    det_success_num = 0  # 用于保存检测成功的图像数量
    for i in range(50):
        image = os.path.join(image_folder, f'{i}.jpg')
        if i == 0:
            print(image)
        if not os.path.exists(image):
            continue
        img = cv2.imread(image)   # 读取图像
        img = cv2.bitwise_not(img)   # 取反，因为棋盘格检测函数findChessboardCorners默认检测黑色角点
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # RGB图像转换为灰度图像

        # 棋盘格检测
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)

        if ret:
            det_success_num += 1
            # 如果成功检测到棋盘格，添加图像平面上的二维点和世界坐标系中的三维点到列表
            obj_points.append(objp)
            img_points.append(corners)

            # 绘制并显示角点
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # 求解标定板位姿
    R_board2cameras = []  # 用于保存旋转矩阵
    t_board2cameras = []  # 用于保存平移向量
    # 迭代的到每张图片相对于相机的位姿
    for i in range(det_success_num):
        # rvec：标定板相对于相机坐标系的旋转向量
        # t_board2camera：标定板相对于相机坐标系的平移向量
        ret, rvec, t_board2camera = cv2.solvePnP(obj_points[i], img_points[i], K, dist_coeffs) 

        # 将旋转向量(rvec)转换为旋转矩阵
        # R：标定板相对于相机坐标系的旋转矩阵
        R_board2camera, _ = cv2.Rodrigues(rvec)   # 输出：R为旋转矩阵和旋转向量的关系  输入：rvec为旋转向量

        # 将标定板相对于相机坐标系的旋转矩阵和平移向量保存到列表
        R_board2cameras.append(R_board2camera)
        t_board2cameras.append(t_board2camera)

    # 求解手眼标定
    R_base2ends, t_base2ends = pose_vectors_to_base2end_transforms(pose_vectors)

    # R_camera2base：相机相对于机械臂基座的旋转矩阵
    # t_camera2base：相机相对于机械臂基座的平移向量
    R_camera2base, t_camera2base = cv2.calibrateHandEye(R_base2ends, t_base2ends, 
                                                        R_board2cameras, t_board2cameras)

    # 将旋转矩阵和平移向量组合成齐次位姿矩阵
    T_camera2base = np.eye(4)
    T_camera2base[:3, :3] = R_camera2base
    T_camera2base[:3, 3] = t_camera2base.reshape(3)

    # 输出相机相对于机械臂基座的旋转矩阵和平移向量
    print("Camera to base / base to camera rotation matrix:")
    print(R_camera2base)
    print("Camera to base translation vector:") 
    print(t_camera2base)

    # 输出相机相对于机械臂基座的位姿矩阵
    print("Camera to base pose matrix:")
    np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
    print(T_camera2base)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Hand-eye calibration using chessboard images.')
    parser.add_argument('pose_file', type=str, help='Path to the CSV file containing pose vectors.')
    parser.add_argument('image_folder', type=str, help='Path to the folder containing chessboard images.')
    args = parser.parse_args()
    main(args.pose_file, args.image_folder)
