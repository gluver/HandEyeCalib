import numpy as np
from hand_to_eye import pose_vectors_to_base2end_transforms
import cv2
import os

def load_ground_truth():
    return np.array([
        [0.99147172, -0.12675601, -0.0302779, 1919.75642688],
        [-0.12609379, -0.99175468, 0.0228694, -769.76075433],
        [-0.03292708, -0.01885651, -0.99927986, 2657.82636401],
        [0.0, 0.0, 0.0, 1.0]
    ])

def matrix_percentage_difference(matrix1, matrix2):
    """
    Calculate percentage difference between two transformation matrices
    Returns (rotation_diff_percent, translation_diff_percent)
    """
    # Extract rotation and translation components
    R1, t1 = matrix1[:3, :3], matrix1[:3, 3]
    R2, t2 = matrix2[:3, :3], matrix2[:3, 3]
    
    # Calculate rotation difference using Frobenius norm
    rotation_diff = np.linalg.norm(R1 - R2, 'fro')
    rotation_base = np.linalg.norm(R1, 'fro')
    rotation_diff_percent = (rotation_diff / rotation_base) * 100
    
    # Calculate translation difference using Euclidean distance
    translation_diff = np.linalg.norm(t1 - t2)
    translation_base = np.linalg.norm(t1)
    translation_diff_percent = (translation_diff / translation_base) * 100
    
    return rotation_diff_percent, translation_diff_percent

def run_calibration_test(pose_data_path, image_dir, pattern_size=(4, 3), square_size=50):
    """
    Run calibration and compare with ground truth
    Returns True if within 1% threshold, False otherwise
    """
    # Load ground truth
    ground_truth = load_ground_truth()
    
    # Load pose data
    pose_vectors = np.loadtxt(pose_data_path, delimiter=',')
    
    # Camera parameters
    camera_matrix = np.array([[1123.9, 0, 982.364],
                            [0, 1123.4, 567.264],
                            [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.0769521, -0.105434, 6.25417e-05, 3.9459e-5, 0.0428337], dtype=np.float32)

    # Create chessboard 3D coordinates
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
    
    # Process images and collect points
    obj_points = []
    img_points = []
    
    for i in range(50):  # Assuming maximum of 50 images
        image_path = os.path.join(image_dir, f"{i}.jpg")
        if not os.path.exists(image_path):
            continue
            
        img = cv2.imread(image_path)
        img = cv2.bitwise_not(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)
        if ret:
            obj_points.append(objp)
            img_points.append(corners)
    
    # Calculate board poses
    R_board2cameras = []
    t_board2cameras = []
    
    for i in range(len(obj_points)):
        ret, rvec, t_board2camera = cv2.solvePnP(obj_points[i], img_points[i], camera_matrix, dist_coeffs)
        R_board2camera, _ = cv2.Rodrigues(rvec)
        R_board2cameras.append(R_board2camera)
        t_board2cameras.append(t_board2camera)
    
    # Convert pose vectors to transforms
    R_base2ends, t_base2ends = pose_vectors_to_base2end_transforms(pose_vectors)
    
    # Perform hand-eye calibration
    R_camera2base, t_camera2base = cv2.calibrateHandEye(R_base2ends, t_base2ends, 
                                                       R_board2cameras, t_board2cameras)
    
    # Create result matrix
    result_matrix = np.eye(4)
    result_matrix[:3, :3] = R_camera2base
    result_matrix[:3, 3] = t_camera2base.reshape(3)
    
    # Calculate differences
    rot_diff, trans_diff = matrix_percentage_difference(ground_truth, result_matrix)
    
    print("Calibration Test Results:")
    print(f"Rotation difference: {rot_diff:.2f}%")
    print(f"Translation difference: {trans_diff:.2f}%")
    print("\nGround Truth Matrix:")
    print(ground_truth)
    print("\nCalculated Matrix:")
    print(result_matrix)
    
    # Test passes if both rotation and translation differences are under 1%
    return rot_diff < 1.0 and trans_diff < 1.0, (rot_diff, trans_diff)

if __name__ == '__main__':
    pose_data_path = 'input_data/filtered_pose_data_2025-02-26_15-25-11.csv'
    image_dir = 'input_data/2025-02-26_15-25-11'
    
    passed, (rot_diff, trans_diff) = run_calibration_test(pose_data_path, image_dir)
    
    if passed:
        print("\nTest PASSED: Calibration results within 1% of ground truth")
        exit(0)
    else:
        print("\nTest FAILED: Calibration results exceed 1% threshold")
        print(f"Rotation difference: {rot_diff:.2f}% (threshold: 1.0%)")
        print(f"Translation difference: {trans_diff:.2f}% (threshold: 1.0%)")
        exit(1)
