import numpy as np
import unittest
import subprocess

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

  
 
  
class TestCalibration(unittest.TestCase):
    def test_hand_to_eye(self):
        result = subprocess.run(['python3', 'hand_to_eye.py', '--config', 'config_template.yaml'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, msg=f"Script failed with output: {result.stdout}\n{result.stderr}")

        # Assuming the script outputs the result matrix in a file or stdout
        result_matrix = np.loadtxt('result_matrix.txt')  # Adjust this line based on actual output method
        ground_truth = load_ground_truth()

        rot_diff, trans_diff = matrix_percentage_difference(result_matrix, ground_truth)

        print("Calibration Test Results:")
        print(f"Rotation difference: {rot_diff:.2f}%")
        print(f"Translation difference: {trans_diff:.2f}%")
        print("\nGround Truth Matrix:")
        print(ground_truth)
        print("\nCalculated Matrix:")
        print(result_matrix)

if __name__ == '__main__':
    unittest.main()
