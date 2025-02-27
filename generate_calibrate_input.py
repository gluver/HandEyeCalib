import os
import shutil
import argparse
from datetime import datetime
import cv2
import numpy as np

def assess_image_quality(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

def main():
    parser = argparse.ArgumentParser(description='Process chessboard images and generate calibration input.')
    parser.add_argument('--src-dir', type=str, required=True,
                      help='Source directory containing the images')
    parser.add_argument('--pose-data', type=str, required=True,
                      help='Path to the pose data CSV file')
    parser.add_argument('--dst-base', type=str, default='./input_data',
                      help='Base directory for output (default: ./input_data)')
    parser.add_argument('--error-threshold', type=float, default=0.9,
                      help='Maximum allowed reprojection error (default: 0.9)')
    parser.add_argument('--chessboard-width', type=int, default=4,
                      help='Number of inner corners along width (default: 4)')
    parser.add_argument('--chessboard-height', type=int, default=3,
                      help='Number of inner corners along height (default: 3)')
    
    args = parser.parse_args()

    # Camera intrinsic parameters
    camera_matrix = np.array([[1123.9, 0, 982.364],
                            [0, 1123.4, 567.264],
                            [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.0769521, -0.105434, 0.0428337, 0, 0, 0, 6.25417e-05, 3.9459e-5], dtype=np.float32)

    # Create base directory if it doesn't exist
    os.makedirs(args.dst_base, exist_ok=True)
    human_friendly_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    dst_dir = os.path.join(args.dst_base, human_friendly_name)

    # Create destination directory if it doesn't exist
    os.makedirs(dst_dir, exist_ok=True)

    # Load pose data
    pose_data = np.loadtxt(args.pose_data, delimiter=',')
    print("Pose data loaded successfully")

    # Get list of image files in the source directory
    image_files = [f for f in os.listdir(args.src_dir) if f.endswith('.jpg')]

    # Sort image files based on timestamp in their names
    image_files.sort(key=lambda x: datetime.strptime(x, 'frame%Y-%m-%d_%H-%M-%S.jpg'))
    
    print(f'Destination directory: {dst_dir}')
    print(f'Source directory: {args.src_dir}')

    chessboard_size = (args.chessboard_width, args.chessboard_height)
    count_passed = 0
    filtered_pose_data = []

    for i, filename in enumerate(image_files, start=0):
        src_path = os.path.join(args.src_dir, filename)
        dst_path = os.path.join(dst_dir, f'{count_passed}.jpg')

        image = cv2.imread(src_path)
        image = cv2.bitwise_not(image)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, chessboard_size, None)

        # Solve for the rotation and translation vectors
        _, rvecs, tvecs = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)

        # Project the object points back to the image plane
        img_points, _ = cv2.projectPoints(objp, rvecs, tvecs, camera_matrix, dist_coeffs)

        # Calculate the reprojection error
        error = cv2.norm(corners, img_points, cv2.NORM_L2) / len(img_points)

        print(f'Reprojection error for {filename}: {error}')
        # if error < 0.3:
        #     # Copy the image to the output directory
        #     if count_passed in [3,4,5,6,15,16]:
        #         dst_path = os.path.join(dst_dir, f'{real_count}.jpg')
        #         shutil.copy(src_path, dst_path)
        #         filtered_pose_data.append(pose_data[i])
        #         print(f'Appended pose data for {filename}: {pose_data[i]}')
        #         quality_score = assess_image_quality(image)
        #         print(f'Renamed {filename} to {real_count}.jpg with quality score: {quality_score}')
        #         real_count+=1
        #     count_passed+=1
        if error < 0.9:
            # Copy the image to the output directory
            shutil.copy(src_path, dst_path)
            filtered_pose_data.append(pose_data[i])
            print(f'Appended pose data for {filename}: {pose_data[i]}')
            quality_score = assess_image_quality(image)
            print(f'Renamed {filename} to {count_passed}.jpg with quality score: {quality_score}')
            count_passed+=1


# Write the filtered pose data to a CSV file
filtered_pose_data = np.array(filtered_pose_data)
pose_output_path = os.path.join(dst_base, f'filtered_pose_data_{human_friendly_name}.csv')
np.savetxt(pose_output_path, filtered_pose_data, delimiter=',')
print(f'Filtered pose data saved to {pose_output_path}')
   