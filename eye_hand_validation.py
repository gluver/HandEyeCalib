import numpy as np
import open3d as o3d
import argparse

def transform_points_to_robot_base_frame(points, cam2base_matrix):
    points = np.hstack((points, np.ones((points.shape[0], 1))))  # Add a column of ones to the points
    points = points.T  # Transpose the points
    points = np.dot(cam2base_matrix, points)  # Perform the transformation
    points = points[:3, :].T  # Take the first three rows and transpose the result
    return points

def load_matrix_from_file(file_path):
    return np.loadtxt(file_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Transform points to robot base frame.')
    parser.add_argument('--points_dir', type=str, default='DepthPoints_1740557933994.ply', help='Path to the PLY file containing points.')
    parser.add_argument('--matrix_file', type=str, default=None, help='Path to the file containing cam2base matrix.')
    args = parser.parse_args()

    if args.matrix_file:
        cam2base_matrix = load_matrix_from_file(args.matrix_file)
    else:
        cam2base_matrix = np.array([
            [0.99147172, -0.12675601, -0.0302779, 1919.75642688],
            [-0.12609379, -0.99175468, 0.0228694, -769.76075433],
            [-0.03292708, -0.01885651, -0.99927986, 2657.82636401],
            [0.0, 0.0, 0.0, 1.0]
        ])
    if args.points_dir == 'DepthPoints_1740557933994.ply':
        print("\033[91mWarning: Using default points file 'DepthPoints_1740557933994.ply'\033[0m")
    if args.matrix_file is None:
        print("\033[91mWarning: Using default cam2base matrix\033[0m")
        
    pcd = o3d.io.read_point_cloud(args.points_dir)
    points = np.asarray(pcd.points)
    print("points converted")
    points = transform_points_to_robot_base_frame(points, cam2base_matrix)
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud("points_in_robot_base_frame.ply", pcd)
