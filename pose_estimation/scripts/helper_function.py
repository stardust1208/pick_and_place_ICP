from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import math
import numpy as np
import copy

class RigidTransform:
    def __init__(self, rotation=np.eye(3), translation=np.zeros((3, 1))):
        self.rotation = rotation
        self.translation = translation
    def multiply(self, points):
        """Áp dụng phép biến đổi vào một tập hợp điểm."""
        return np.dot(self.rotation, points) + self.translation
    def multiplyRigid(self, other):
        """Returns a new RigidTransform that is the result of applying this transformation followed by another."""
        new_rotation = self.rotation @ other.rotation  # Matrix multiplication for rotations
        new_translation = self.rotation @ other.translation + self.translation  # Apply rotation and then translate
        return RigidTransform(new_rotation, new_translation)

class ICPTransform():
    def __init__(self, x_length, y_length, z_length, offset, step, max_iter, gaussian_noise=0.005):
        self.x_length = x_length
        self.y_length = y_length
        self.z_length = z_length
        self.offset = offset
        self.step = step
        self.max_iterations = max_iter
        self.gaussian_noise = gaussian_noise
        self.initial_X_BA = None
        self.model = self.create_uniform_box_points()

    def rotation_matrix_to_euler(self, rotation_matrix):
        rotation = R.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        return euler_angles

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        rotation_matrix = rotation.as_matrix()
        return rotation_matrix

    def create_uniform_plane(self, case):
        points = []
        step_x = self.x_length/math.ceil(self.x_length/self.step)
        step_y = self.y_length/math.ceil(self.y_length/self.step)   
        step_z = self.z_length/math.ceil(self.z_length/self.step)

        x_points = np.arange(-self.x_length/2, self.x_length/2 + step_x - 1e-8, step_x)
        y_points = np.arange(-self.y_length/2, self.y_length/2 + step_y - 1e-8, step_y)
        z_points = np.arange(-self.z_length/2, self.z_length/2 + step_z - 1e-8, step_z)

        if case == 3:
            # for z in [-self.z_length/2, self.z_length/2]:
            z = -self.z_length/2
            X, Y = np.meshgrid(x_points, y_points)
            face_points = np.column_stack([X.flatten(), Y.flatten(), np.full_like(X.flatten(), z)])
            points.append(face_points)
        elif case == 2:
            # for y in [-self.y_length/2, self.y_length/2]:
            y = -self.y_length/2
            X, Z = np.meshgrid(x_points, z_points)
            face_points = np.column_stack([X.flatten(), np.full_like(X.flatten(), y), Z.flatten()])
            points.append(face_points)
        elif case == 1:
            # for x in [-self.x_length/2, self.x_length/2]:
            x = -self.x_length/2
            Y, Z = np.meshgrid(y_points, z_points)
            face_points = np.column_stack([np.full_like(Y.flatten(), x), Y.flatten(), Z.flatten()])
            points.append(face_points)
        return np.vstack(points).T

    def create_uniform_box_points(self):
        """
        Tạo điểm đều trên các mặt của hình hộp
        Parameters:
            length: chiều dài (trục x)
            width: chiều rộng (trục y)
            height: chiều cao (trục z)
            step: khoảng cách giữa các điểm
        
        Returns:
            points: mảng numpy (N, 3) chứa các điểm 3D
        """
        points = []
        step_x = self.x_length/math.ceil(self.x_length/self.step)
        step_y = self.y_length/math.ceil(self.y_length/self.step)   
        step_z = self.z_length/math.ceil(self.z_length/self.step)

        x_points = np.arange(-self.x_length/2, self.x_length/2 + step_x - 1e-8, step_x)
        y_points = np.arange(-self.y_length/2, self.y_length/2 + step_y - 1e-8, step_y)
        for z in [-self.z_length/2, self.z_length/2]:
            X, Y = np.meshgrid(x_points, y_points)
            face_points = np.column_stack([X.flatten(), Y.flatten(), np.full_like(X.flatten(), z)])
            points.append(face_points)
        
        z_points = np.arange(-self.z_length/2 + step_z, self.z_length/2 - 1e-8, step_z)
        for y in [-self.y_length/2, self.y_length/2]:
            X, Z = np.meshgrid(x_points, z_points)
            face_points = np.column_stack([X.flatten(), np.full_like(X.flatten(), y), Z.flatten()])
            points.append(face_points)

        y_points = np.arange(-self.y_length/2 + step_y, self.y_length/2 - 1e-8, step_y)
        for x in [-self.x_length/2, self.x_length/2]:
            Y, Z = np.meshgrid(y_points, z_points)
            face_points = np.column_stack([np.full_like(Y.flatten(), x), Y.flatten(), Z.flatten()])
            points.append(face_points)
        return np.vstack(points).T

    def transform_model(self, translation, rotation, model):
        """
        Transform model by translation and euler angle
        """
        x, y, z = translation
        roll, pitch, yaw = rotation
        # Tính toán ma trận quay
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        # Ma trận quay tổng hợp
        R = R_z @ R_y @ R_x
        # Tạo ma trận biến đổi thuần nhất 4x4
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = [x, y, z]
        # Chuyển đổi ma trận model thành dạng đồng nhất
        n = model.shape[1]  # Số cột của ma trận model
        homogeneous_model = np.vstack((model, np.ones(n)))
        # Thực hiện phép biến đổi
        new_homogeneous_model = transformation_matrix @ homogeneous_model
        # Lấy lại ma trận model mới 3xN
        new_model = new_homogeneous_model[:3, :]
        return new_model

    def nearest_neighbors(self, scene, new_model):
        kdtree = KDTree(new_model.T)
        distances, indices = kdtree.query(scene.T, k=1)
        return distances.flatten(), indices.flatten()

    def least_squares_transform(self, scene, new_model) -> RigidTransform:
        X_BA = RigidTransform()
        length = new_model.shape[1]
        pm = np.mean(new_model, axis=1, keepdims=True)  
        ps = np.mean(scene, axis=1, keepdims=True)
        W = np.zeros((3, 3))
        for i in range(length):
            scene3x1 = scene[:, i].reshape(-1, 1)
            model1x3 = new_model[:, i].reshape(1, -1)
            add = np.dot((scene3x1 - ps), (model1x3 - pm.T))
            W = W + add
        U, S, VT = np.linalg.svd(W)
        detUVT = np.sign(np.linalg.det(np.dot(U, VT)))
        D = np.zeros((3, 3))
        D = np.diag([1, 1, detUVT])
        Rstar = np.dot(np.dot(U, D), VT)
        pstar = ps - np.dot(Rstar, pm)
        X_BA = RigidTransform(rotation=Rstar, translation=pstar)
        return X_BA

    def icp(self, scene, case, tolerance=1e-7):
        X_BA = self.get_initial_X_BA(case)
        initial_X_BA = copy.deepcopy(X_BA)
        mean_error=0
        num_iters=0
        prev_error=0
        while True:
            num_iters += 1
            new_model = X_BA.multiply(self.model)
            distances, indices = self.nearest_neighbors(scene, new_model)
            modified_model = new_model[:, indices]
            new_X_BA = self.least_squares_transform(scene, modified_model)
            X_BA = new_X_BA.multiplyRigid(X_BA)
            mean_error = np.mean(distances)
            if abs(mean_error - prev_error) < tolerance:
                # print("mean")
                break
            if num_iters >= self.max_iterations:
                # print("iter")
                break
            prev_error = mean_error
        return X_BA, initial_X_BA, mean_error, num_iters

    def get_initial_X_BA(self, case) -> RigidTransform:
        initial_X_BA = RigidTransform()
        if case == 1:
            initial_X_BA.translation = np.array([[0], [0], [self.offset - self.x_length/2]])
            initial_X_BA.rotation = self.euler_to_rotation_matrix(0, np.pi/2, 0)
        if case == 2:
            initial_X_BA.translation = np.array([[0], [0], [self.offset - self.y_length/2]])
            initial_X_BA.rotation = self.euler_to_rotation_matrix(np.pi/2, 0, 0)
        if case == 3:
            initial_X_BA.translation = np.array([[0], [0], [self.offset - self.z_length/2]])
            initial_X_BA.rotation = self.euler_to_rotation_matrix(0, 0, 0)
        return initial_X_BA

    def ICP(self, scene, case):
        # Áp dụng ICP để tính phép biến đổi giữa scene và model
        X_BA, initia_X_BA, err, _ = self.icp(scene, case)

        center = X_BA.multiply(np.array([[0], [0], [0]]))
        x_unit = X_BA.multiply(np.array([[0.2], [0], [0]]))
        y_unit = X_BA.multiply(np.array([[0], [0.2], [0]]))
        z_unit = X_BA.multiply(np.array([[0], [0], [0.2]]))

        modified = RigidTransform()
        if case == 1 and x_unit[2]-center[2] < 0:
            modified.rotation = np.array([[-1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, -1]])
            X_BA = X_BA.multiplyRigid(modified)
        elif case == 2 and y_unit[2]-center[2] < 0:
            modified.rotation = np.array([[1, 0, 0],
                                        [0, -1, 0],
                                        [0, 0, -1]])
            X_BA = X_BA.multiplyRigid(modified)
        elif case == 3 and z_unit[2]-center[2] < 0:
            modified.rotation = np.array([[-1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, -1]])
            X_BA = X_BA.multiplyRigid(modified)
        return X_BA, initia_X_BA, err

    def angles_between_axes(self, rotation):
        """
        Calculates the angles between the x, y, z axes of the object frame and the z-axis of the world frame.
        Args:
        - T (numpy.ndarray): A 4x4 transformation matrix (pose) representing the object frame in world coordinates.
        Returns:
        - angles (dict): A dictionary containing the angles (in radians) between each axis of the object frame
                        and the z-axis of the world frame.
                        Keys are 'x', 'y', 'z', representing the respective axes.
        """
        # Z-axis of the world frame (fixed, pointing in the positive z direction)
        z_world = np.array([0, 0, 1])
        
        # Extract the x, y, and z axes of the object frame from the rotation part of the transformation matrix
        x_object = rotation[:3, 0]  # First column: x-axis of the object frame
        y_object = rotation[:3, 1]  # Second column: y-axis of the object frame
        z_object = rotation[:3, 2]  # Third column: z-axis of the object frame
        
        # Function to calculate the angle between two vectors
        def angle_between(v1, v2):
            cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_theta = np.clip(cos_theta, -1.0, 1.0)  # To handle floating point errors
            return np.arccos(cos_theta)
        
        # Calculate angles between object axes and world z-axis
        angle_x = angle_between(x_object, z_world)
        angle_y = angle_between(y_object, z_world)
        angle_z = angle_between(z_object, z_world)
        
        # Return the angles in radians
        return angle_x, angle_y, angle_z

    def align_axis_to_z(self, R_obj):
        """
        Align the axis of the object that has the minimum angle with the z-axis of the world frame to the z-axis.
        
        Args:
        - R_obj (numpy.ndarray): A 3x3 rotation matrix representing the object frame in world coordinates.
        
        Returns:
        - R_aligned (numpy.ndarray): A new rotation matrix with the selected axis aligned to the world z-axis.
        """
        # Calculate angles between object axes and world z-axis
        xang, yang, zang = self.angles_between_axes(R_obj)
        
        # Identify the axis with the smallest angle
        min_ange = min(xang, yang, zang)
        if min_ange == xang:
            axis_object = R_obj[:, 0]
        elif min_ange == yang:
            axis_object = R_obj[:, 1]
        else:
            axis_object = R_obj[:, 2]
        
        # Z-axis of the world frame
        z_world = np.array([0, 0, 1])
        
        # Compute the axis of rotation (cross product) and the angle between the two vectors
        axis_of_rotation = np.cross(axis_object, z_world)
        axis_of_rotation = axis_of_rotation / np.linalg.norm(axis_of_rotation)  # Normalize
        angle = np.arccos(np.dot(axis_object, z_world) / (np.linalg.norm(axis_object) * np.linalg.norm(z_world)))
        
        # If the angle is 0 or pi, no rotation is needed
        if np.isclose(angle, 0):
            return R_obj  # No rotation needed
        
        # Create the rotation matrix using Rodrigues' rotation formula
        r = R.from_rotvec(axis_of_rotation * angle)
        R_aligned = r.as_matrix() @ R_obj
        return R_aligned

    def project_pc_to_lower_plane_and_delete_mid_points(self, points, case, offset=0.005):
        """
        Duplicate points cloud in the top plane to the table plane and delete any points between
        """
        if case == 1:
            projected_z = self.x_length
        elif case == 2:
            projected_z = self.y_length
        elif case == 3:
            projected_z = self.z_length
        else:
            return points
        # Extract z values
        z_values = points[2, :]
        # Define the range for projection
        z_min = np.min(z_values)
        z_max = z_min + offset
        # Identify points within the range
        in_range_mask = (z_values >= z_min) & (z_values <= z_max)
        # Filter out points not in range
        points_in_range = points[:, in_range_mask]
        # Duplicate the points in range with an offset applied to the z-coordinate
        duplicated_points = points_in_range.copy()
        duplicated_points[2, :] += projected_z
        # Combine only the duplicated points
        new_points = np.hstack((points_in_range, duplicated_points))
        return new_points

    def project_pc_to_lower_plane(self, points, case, offset = 0.005):
        """
        Duplicate points cloud in the top plane to the table plane
        """
        if case == 1:
            projected_z = self.x_length
        elif case == 2:
            projected_z = self.y_length
        elif case == 3:
            projected_z = self.z_length
        else:
            return points
        # Extract z values
        z_values = points[2, :]
        # Define the range for projection
        z_min = np.min(z_values)
        z_max = z_min + offset
        # Identify points within the range
        in_range_mask = (z_values >= z_min) & (z_values <= z_max)
        # Select points to be projected
        points_to_project = points[:, in_range_mask]
        # Project the selected points to the target_z
        projected_points = points_to_project.copy()
        projected_points[2, :] = points_to_project[2, :] + projected_z
        # Combine original points and projected points
        new_points = np.hstack((points, projected_points))
        return new_points


if __name__ ==  '__main__':
    offset = 0.44
    process = ICPTransform(x_length=0.025, y_length=0.06, z_length=0.12, offset=0.44, step=0.005)
    case = 3
    model = process.create_uniform_box_points()

    #########################################
    # chi chay cai nay khi test
    x_length, y_length, z_length, step = 0.025, 0.06, 0.12, 0.008
    xyz = [0.24, 0.3, offset]
    rpy = [np.radians(0), np.radians(90), np.radians(50)]
    noise = np.random.normal(0, 0.001, model.shape)
    if case == 1:
        translation = (xyz[0], xyz[1], xyz[2] - x_length/2)
        rotation = (rpy[0], rpy[1], rpy[2])
    elif case == 2:
        translation = (xyz[0], xyz[1], xyz[2] - y_length/2)
        rotation = (rpy[1], rpy[0], rpy[2])
    else:
        translation = (xyz[0], xyz[1], xyz[2] - z_length/2)
        rotation = (rpy[0], rpy[0], rpy[2])
    # scene = process.transform_model(translation, rotation, model) + noise
    plane = process.create_uniform_plane(case)
    scene = process.transform_model(translation, rotation, plane) + np.random.normal(0, 0.002, plane.shape)
    scene = process.project_pc_to_lower_plane(scene, case)
    #########################################


    X_BA, initial_X_BA, err = process.ICP(scene, case)
    xang, yang, zang = process.angles_between_axes(X_BA.rotation)


    print("################################")
    print(X_BA.rotation)
    print(X_BA.translation)
    print(process.rotation_matrix_to_euler(X_BA.rotation))
    print("################################")
    print(rotation)
    print(translation)
    print("################################")
    print(np.degrees(xang), np.degrees(yang), np.degrees(zang))

    center = X_BA.multiply(np.array([[0], [0], [0]]))
    x_unit = X_BA.multiply(np.array([[0.2], [0], [0]]))
    y_unit = X_BA.multiply(np.array([[0], [0.2], [0]]))
    z_unit = X_BA.multiply(np.array([[0], [0], [0.2]]))
    pose = X_BA.multiply(model)
    estimated_pose = initial_X_BA.multiply(model)
    xx = [center[0, 0], x_unit[0, 0]]
    xy = [center[1, 0], x_unit[1, 0]]
    xz = [center[2, 0], x_unit[2, 0]]
    yx = [center[0, 0], y_unit[0, 0]]
    yy = [center[1, 0], y_unit[1, 0]]
    yz = [center[2, 0], y_unit[2, 0]]
    zx = [center[0, 0], z_unit[0, 0]]
    zy = [center[1, 0], z_unit[1, 0]]
    zz = [center[2, 0], z_unit[2, 0]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(model[0, :], model[1, :], model[2, :], c='green')
    ax.scatter(scene[0, :], scene[1, :], scene[2, :], c='red')
    # ax.scatter(pose[0, :], pose[1, :], pose[2, :], c='cyan')
    ax.scatter(estimated_pose[0, :], estimated_pose[1, :], estimated_pose[2, :], c='blue')
    
    ax.plot(xx, xy, xz, color="red")
    ax.plot(yx, yy, yz, color="green")
    ax.plot(zx, zy, zz, color="blue")

    ax.set_xlabel('$X (m)$')
    ax.set_ylabel('$Y (m)$')
    ax.set_zlabel('$Z (m)$')
    ax.set_xlim([-0.1, 0.5])
    ax.set_ylim([-0.1, 0.5])
    ax.set_zlim([-0.1, 0.5])

    plt.show()
