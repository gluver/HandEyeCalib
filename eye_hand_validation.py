import numpy as np
import open3d as o3d 
#given an array with 3D points in the camera point cloud (peform the transformation to the robot base frame)
#and return the transformed 3D points in the robot base frame
cam2base_matrix = np.array([
    [0.99147172, -0.12675601, -0.0302779, 1919.75642688],
    [-0.12609379, -0.99175468, 0.0228694, -769.76075433],
    [-0.03292708, -0.01885651, -0.99927986, 2657.82636401],
    [0.0, 0.0, 0.0, 1.0]
])


def transform_points_to_robot_base_frame(points):
    points = np.hstack((points, np.ones((points.shape[0], 1))))  # Add a column of ones to the points
    points = points.T  # Transpose the points
    points = np.dot(cam2base_matrix, points)  # Perform the transformation
    points = points[:3, :].T  # Take the first three rows and transpose the result
    return points
#load from a ply file the 3D points in the camera
def load_camera_point_cloud(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd.points


if __name__ == '__main__':
    # user give the path of the ply file or manually give the points array
    points_dir='DepthPoints_1740557933994.ply'
    pcd = o3d.io.read_point_cloud(points_dir)
    points=np.asarray(pcd.points)
    print("points converted")
    points=transform_points_to_robot_base_frame(points)

    pcd.points = o3d.utility.Vector3dVector(points)
    #when hovering on a point we can see the coordinates
    o3d.io.write_point_cloud("points_in_robot_base_frame.ply", pcd)
    


