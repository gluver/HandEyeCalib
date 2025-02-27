import os
import shutil
import argparse
from datetime import datetime
import cv2
import numpy as np
import yaml

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
    image_files = [f for f in os.listdir(config['src_dir']) if f.endswith('.jpg')]

    # Sort image files based on timestamp in their names
    image_files.sort(key=lambda x: datetime.strptime(x, 'frame%Y-%m-%d_%H-%M-%S.jpg'))
    
    print(f'Destination directory: {dst_dir}')
    print(f"Source directory: {config['src_dir']}")

    chessboard_pattern_size = config["pattern_size"]
    count_passed = 0
    filtered_pose_data = []

    # Define the objp variable
    objp = np.zeros((config['pattern_size'][0] * config['pattern_size'][1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:config['pattern_size'][0], 0:config['pattern_size'][1]].T.reshape(-1, 2)

    for i, filename in enumerate(image_files, start=0):
        src_path = os.path.join(config['src_dir'], filename)
        dst_path = os.path.join(dst_dir, f'{count_passed}.jpg')

        image = cv2.imread(src_path)
        image = cv2.bitwise_not(image)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, chessboard_pattern_size, None)
        if not ret:
            print(f"Chessboard not found in {filename}")
            continue
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
            print(f'Renamed {filename} to {count_passed}.jpg with quality score: {quality_score}')
            count_passed += 1

    # Write the filtered pose data to a CSV file
    filtered_pose_data = np.array(filtered_pose_data)
    pose_output_path = os.path.join(dst_dir, f'filtered_pose_data_{human_friendly_name}.csv')
    np.savetxt(pose_output_path, filtered_pose_data, delimiter=',', fmt='%.6f')
    print(f'Filtered pose data saved to {pose_output_path}')

if __name__ == "__main__":
    main()
