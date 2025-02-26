import os
import shutil
from datetime import datetime
import cv2
import numpy as np

def assess_image_quality(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

# Source and destination directories
src_dir = './result0225/DI0225'
# Base directory for destination
dst_base = './input_data'

pose_data_dir='./result0225/0225.csv'
pose_data = np.loadtxt(pose_data_dir, delimiter=',')
print("Pose data:")
print(pose_data)
chessboard_size = (4, 3)  # Adjust this based on your chessboard pattern

# Camera intrinsic parameters (example values, replace with your actual parameters)
camera_matrix = np.array([[1123.9, 0, 982.364],
                          [0, 1123.4, 567.264],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.0769521, -0.105434, 0.0428337, 0, 0, 0, 6.25417e-05, 3.9459e-5], dtype=np.float32)


# Create base directory if it doesn't exist
os.makedirs(dst_base, exist_ok=True)
human_friendly_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
dst_dir = os.path.join(dst_base,human_friendly_name)

# Create destination directory if it doesn't exist
os.makedirs(dst_dir, exist_ok=True)

# Get list of image files in the source directory
image_files = [f for f in os.listdir(src_dir) if f.endswith('.jpg')]

# Sort image files based on timestamp in their names
image_files.sort(key=lambda x: datetime.strptime(x, 'frame%Y-%m-%d_%H-%M-%S.jpg'))
# Log the destination directory name
print(f'Destination directory: {dst_dir}')
# Log the source directory name
print(f'Source directory: {src_dir}')
# Copy and rename images to the destination directory
count_passed=0
real_count=0
filtered_pose_data=[]
for i, filename in enumerate(image_files, start=0):

    src_path = os.path.join(src_dir, filename)
    dst_path = os.path.join(dst_dir, f'{count_passed}.jpg')
    # Find chessboard corners
    image = cv2.imread(src_path)
    image = cv2.bitwise_not(image)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray_image, chessboard_size, None)
    if ret:
        # Draw and display the corners
        cv2.drawChessboardCorners(image, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', image)
        cv2.waitKey(100)  # Display for 500 ms
        cv2.destroyAllWindows()

        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

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
   