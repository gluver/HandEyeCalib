
import cv2
import numpy as np
import transforms3d
import glob
import re
import os
import argparse
import yaml
import logging

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

def load_config(config_file):
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config

def main(config_file):
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    
    config = load_config(config_file)

    # 输入位姿数据
    pose_vectors = np.loadtxt(config['pose_file'], delimiter=',')
    logger.info(f"Loaded pose vectors from CSV:{config['pose_file']}")
    # logger.info(pose_vectors)

    # 导入相机内参和畸变参数
    camera_params = config['camera_params']
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = camera_params
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
    K = camera_matrix

    # 棋盘格参数
    square_size = config['square_size']
    pattern_size = tuple(config['pattern_size'])
    corner_color = config['corner_color']

    # 准备位姿数据
    obj_points = []  # 用于保存世界坐标系中的三维点
    img_points = []  # 用于保存图像平面上的二维点

    # 创建棋盘格3D坐标
    objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    # 迭代处理图像
    det_success_num = 0  # 用于保存检测成功的图像数量
    for i in range(50):
        image = os.path.join(config['image_folder'], f'{i}.jpg')
        if i == 0:
            logger.info(f"Using input image from {config['image_folder']}")
        if not os.path.exists(image):
            continue
        img = cv2.imread(image)   # 读取图像
        if corner_color == 'black':
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
            cv2.waitKey(100)

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
    logger.info("Camera to base rotation matrix:")
    logger.info(R_camera2base)
    logger.info("Camera to base translation vector:") 
    logger.info(t_camera2base)

    # 输出相机相对于机械臂基座的位姿矩阵
    logger.info("Camera to base pose matrix:")
    np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
    logger.info(T_camera2base)
    # 将位姿矩阵写入文件
    result_file = config.get('result_file', 'result_matrix.txt')
    with open(result_file, 'w') as f:
        for line in T_camera2base:
            np.savetxt(f, line[np.newaxis], fmt='%.6f')
    logger.info(f"Camera to base pose matrix saved to {result_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Hand-eye calibration using chessboard images.')
    parser.add_argument('--config_file', type=str, default='hand_to_eye_config_template.yaml', help='Path to the YAML configuration file.')
    args = parser.parse_args()
    if args.config_file=='hand_to_eye_config_template.yaml':
            logging.basicConfig(level=logging.INFO)
            logger = logging.getLogger(__name__)
            logger.warning("\033[91mUsing default configuration template. The parameter settings inside may not adapt to your current setup.\033[0m")

    main(args.config_file)
