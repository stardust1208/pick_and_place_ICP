import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

roll, pitch, yaw = np.radians([0, 90, -130])



rpy = [roll, pitch, yaw]
rotation_matrix = R.from_euler('xyz', rpy).as_matrix()  # 3x3 rotation matrix
z_axis_obj = rotation_matrix[:, 2]  # Third column of rotation matrix (Z-axis of obj)
x_axis_base = np.array([1, 0, 0])  # X-axis of base_link
cross_product = np.cross(x_axis_base, z_axis_obj)  # Cross product gives rotation direction
dot_product = np.dot(x_axis_base, z_axis_obj)
roll_obj = np.arccos(dot_product)
if cross_product[2] < 0:
    roll_obj = -roll_obj
if roll_obj < -np.pi/2:
    roll_obj = roll_obj + np.pi
elif roll_obj > np.pi/2:
    roll_obj = roll_obj - np.pi
print (roll_obj*180/np.pi)



fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')

# Create a rotation matrix from RPY
r = R.from_euler('xyz', [roll, pitch, yaw])
rot_matrix = r.as_matrix()

# Define base coordinate frame (before rotation)
origin = np.array([0, 0, 0])
axes = np.eye(3)  # Identity matrix (X, Y, Z unit vectors)

# Apply rotation to axes
rotated_axes = rot_matrix @ axes

# Plot original (reference) axes in gray
colors = ['red', 'green', 'blue']
labels = ['X', 'Y', 'Z']
for i in range(3):
    ax.quiver(*origin, *axes[:, i], color='gray', linestyle='dashed', alpha=0.5)
    ax.text(*axes[:, i] * 1.2, labels[i] + ' (ref)', color='gray')

# Plot rotated axes
for i in range(3):
    ax.quiver(*origin, *rotated_axes[:, i], color=colors[i])
    ax.text(*rotated_axes[:, i] * 1.2, labels[i], color=colors[i])

# Set limits and labels
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title(f"RPY Rotation: Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°, Yaw={np.degrees(yaw):.1f}°")

plt.show()