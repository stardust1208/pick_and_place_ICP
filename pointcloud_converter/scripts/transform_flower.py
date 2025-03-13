import numpy as np
import open3d as o3d

def transform_points(points_A, T_WA):
    """
    Transform 3xN points from frame A to world frame using T_WA.
    
    Args:
        points_A (numpy.ndarray): 3xN array of points in frame A.
        T_WA (numpy.ndarray): 4x4 transformation matrix from frame A to world frame.
        
    Returns:
        numpy.ndarray: 3xN array of points in the world frame.
    """
    # Convert points_A to homogeneous coordinates
    ones = np.ones((1, points_A.shape[1]))
    points_A_homogeneous = np.vstack((points_A, ones))  # 4xN
    
    # Apply the transformation
    points_W_homogeneous = T_WA @ points_A_homogeneous  # 4xN
    
    # Convert back to 3D coordinates
    points_W = points_W_homogeneous[:3, :]  # 3xN
    
    return points_W

if __name__ == '__main__':
    xyzA = np.load('pointsPose2.npy')
    rgbA = np.load('pointsColor2.npy')
    xyzB = np.load('pointsPose.npy')
    rgbB = np.load('pointsColor.npy')

    # T_WA = np.array([
    #     [0, -1, 0, 0],  # Rotation + translation
    #     [0, 0, -1, 0],
    #     [1, 0, 0, 0.12],
    #     [0, 0, 0, 1]
    # ])  # Example transformation matrix
    T_WA = np.array([
        [0, 0, 1, 0],  # Rotation + translation
        [-1, 0, 0, 0],
        [0, -1, 0, 0.12],
        [0, 0, 0, 1]
    ])  # Example transformation matrix
    # T_WB = np.array([
    #     [0, 1, 0, 0.54],  # Rotation + translation
    #     [0, 0, -1, 0],
    #     [-1, 0, 0, 0.12],
    #     [0, 0, 0, 1]
    # ])  # Example transformation matrix
    T_WB = np.array([
        [0, 0, -1, 0.55],  # Rotation + translation
        [1, 0, 0, 0],
        [0, -1, 0, 0.12],
        [0, 0, 0, 1]
    ])  # Example transformation matrix

    xyzAW = transform_points(xyzA, T_WA)
    xyzBW = transform_points(xyzB, T_WB)

    points = np.concatenate((xyzAW.T, xyzBW.T), axis=0)
    colors = np.concatenate(((rgbA/255.0).T, (rgbB/255.0).T), axis=0)

    # points = xyzBW.T
    # colors = (rgbB/ 255.0).T  # Normalize RGB to range [0, 1] and convert to N x 3

    # Add orgin points and axis xyz according to color rgb
    rootP = np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]])
    rootC = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    points = np.vstack((points, rootP))
    colors = np.vstack((colors, rootC))

    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud], window_name="Point Cloud Visualization")
