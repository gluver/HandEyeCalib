import os
import shutil
import argparse
from datetime import datetime
import cv2
import numpy as np
import yaml
import pickle

np.set_printoptions(suppress=True) 

def assess_image_quality(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

def main():
    parser = argparse.ArgumentParser(description='Process chessboard images and generate calibration input.')
    parser.add_argument('--config-file',default='generate_calibrate_input_config.yaml', type=str,
                      help='Path to the configuration YAML file')
    
    args = parser.parse_args()

    # Load configuration
    with open(args.config_file, 'r') as file:
        config = yaml.safe_load(file)

    # Camera intrinsic parameters
    camera_params = config['camera_params']
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = camera_params
    camera_matrix = np.array([[fx, 0, cx],
                            [0, fy, cy],
                            [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

    # Create base directory if it doesn't exist
    os.makedirs(config['dst_base'], exist_ok=True)
    human_friendly_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    dst_dir = os.path.join(config['dst_base'], human_friendly_name)

    # Create destination directory if it doesn't exist
    os.makedirs(dst_dir, exist_ok=True)

    # Load pose data
    pose_data = np.loadtxt(config['pose_file'], delimiter=',')
    print("Pose data loaded successfully")

    # Get list of image files in the source directory
    image_files = [f for f in os.listdir(config['src_dir']) if f.endswith('.jpg') or f.endswith('.png')]

    # Sort image files based on timestamp in their names
    if config['sort_by_timestamp']:
        image_files.sort(key=lambda x: datetime.strptime(x, 'frame%Y-%m-%d_%H-%M-%S.jpg'))
    else:
        image_files.sort(key=lambda x: int(x.split('_')[0]))
    print(image_files)
    print(f'Destination directory: {dst_dir}')
    print(f"Source directory: {config['src_dir']}")

    chessboard_pattern_size = config["pattern_size"]
    square_size=config['square_size']
    count_passed = 0
    filtered_pose_data = []
    filtered_board2cam_matrices=[]
    filtered_base2gripper_matrices=[]
    # Define the objp variable
    objp = np.zeros((config['pattern_size'][0] * config['pattern_size'][1], 3), np.float32)
    #??  
    objp[:, :2] = np.mgrid[0:config['pattern_size'][0], 0:config['pattern_size'][1]].T.reshape(-1, 2)*square_size

    for i, filename in enumerate(image_files, start=0):
        src_path = os.path.join(config['src_dir'], filename)
        file_extension = os.path.splitext(filename)[1]
        dst_path = os.path.join(dst_dir, f'{count_passed}{file_extension}')

        image = cv2.imread(src_path)
        if config['corner_color'] == 'black':
            image = cv2.bitwise_not(image)
       
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, chessboard_pattern_size, None)
        if not ret:
            print(f"Chessboard not found in {filename}")
            continue
        if ret:
            # 绘制并显示角点
            cv2.drawChessboardCorners(image, config['pattern_size'], corners, ret)
            cv2.imshow('img', image)
            cv2.waitKey(500)
    
        # Solve for the rotation and translation vectors
        _, rvecs, tvecs = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)

        # Project the object points back to the image plane
        img_points, _ = cv2.projectPoints(objp, rvecs, tvecs, camera_matrix, dist_coeffs)

        # Calculate the reprojection error
        error = cv2.norm(corners, img_points, cv2.NORM_L2) / len(img_points)

        print(f'Reprojection error for {filename}: {error}')
        if error < config['error_threshold']:  
            # Copy the image to the output directory
            shutil.copy(src_path, dst_path)
            filtered_pose_data.append(pose_data[i])
            print(f'Appended pose data for {filename}: {pose_data[i]}')
            quality_score = assess_image_quality(image)
            print(f'Renamed {filename} to {count_passed}.jpg with quality score: {quality_score} and reprojection error: {error}')
            # Form the board2cam matrix
            rmat, _ = cv2.Rodrigues(rvecs)
            tvecs = tvecs # Convert translation vectors from mm to meters
            board2cam_matrix = np.hstack((rmat, tvecs))
            board2cam_matrix = np.vstack((board2cam_matrix, [0, 0, 0, 1]))
            filtered_board2cam_matrices.append(board2cam_matrix)
            print(f'board2cam matrix for {filename}: \n{board2cam_matrix}')
            count_passed += 1
            if config['save_base2gripper_4dof']:
                # form a base2gripper matrix from the pose data lines x,y,z,rz (the u in epison scara , and x,y,z in mm)
                x, y, z, rz = pose_data[i]
                base2gripper_matrix = np.array([
                    [np.cos(rz), -np.sin(rz), 0, x],
                    [np.sin(rz), np.cos(rz), 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1]
                ])
                filtered_base2gripper_matrices.append(base2gripper_matrix)
                print(f'Appended base2gripper matrix for {filename}: \n {base2gripper_matrix}')

    # Write the filtered pose data to a CSV file
    filtered_pose_data = np.array(filtered_pose_data)
    pose_output_path = os.path.join(dst_dir, f'filtered_pose_data_{human_friendly_name}.csv')
    np.savetxt(pose_output_path, filtered_pose_data, delimiter=',', fmt='%.6f')
    print(f'Filtered pose data saved to {pose_output_path}')
    # filtered_board2cam_matrices=np.array(filtered_board2cam_matrices)
    # print(f'Shape of filtered pose data: {filtered_pose_data.shape}')
    # print(f'Shape of filtered board2cam matrices: {filtered_board2cam_matrices.shape}')
    board2cam_output_path = os.path.join(dst_dir, f'filtered_board2cam_matrices_{human_friendly_name}.pkl')
    with open(board2cam_output_path, 'wb') as f:
        pickle.dump(filtered_board2cam_matrices, f)
    print(f'Filtered board2cam matrices saved to {board2cam_output_path}')

    # filtered_base2gripper_matrices = np.array(filtered_base2gripper_matrices)
    # print(f'Shape of filtered base2gripper matrices: {filtered_base2gripper_matrices.shape}')
    base2gripper_output_path = os.path.join(dst_dir, f'filtered_base2gripper_matrices_{human_friendly_name}.pkl')
    with open(base2gripper_output_path, 'wb') as f:
        pickle.dump(filtered_base2gripper_matrices, f)
    print(f'Filtered base2gripper matrices saved to {base2gripper_output_path}')
    # save the (base2gripper_matrices ,target2cam_matrices) tuple to a pickel as well with name paired_poses.pkl
    paired_poses_output_path = os.path.join(dst_dir, f'paired_poses_{human_friendly_name}.pkl')
    with open(paired_poses_output_path, 'wb') as f:
        pickle.dump((filtered_base2gripper_matrices,filtered_board2cam_matrices), f)
    print(f'Paired poses saved to {paired_poses_output_path}')
if __name__ == "__main__":
    main()
