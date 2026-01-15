import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# Initialize figure and 3D axes
fig = plt.figure(figsize=(12, 6))
ax1 = fig.add_subplot(121, projection='3d', title="Euler Angles (Gimbal Lock)")
ax2 = fig.add_subplot(122, projection='3d', title="Quaternions (No Gimbal Lock)")

# Set up 3D axes limits
for ax in [ax1, ax2]:
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

# Create a unit cube to visualize orientation
cube_vertices = np.array([
    [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
    [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]
])
edges = [
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5),
    (2, 3), (2, 6), (3, 7), (4, 5), (4, 6),
    (5, 7), (6, 7)
]

# Plot initialization for animation
lines_euler = [ax1.plot([], [], [], 'b')[0] for _ in edges]
lines_quaternion = [ax2.plot([], [], [], 'g')[0] for _ in edges]

# Animation update function
def update(frame):
    # Euler Angles
    pitch = frame * 2  # Animate pitch rotation
    roll = 0
    yaw = 0
    rot_euler = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    rotated_vertices_euler = rot_euler.apply(cube_vertices)
    
    # Quaternions
    q = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    rot_quaternion = R.from_quat(q)
    rotated_vertices_quaternion = rot_quaternion.apply(cube_vertices)
    
    # Update Euler lines
    for edge, line in zip(edges, lines_euler):
        start, end = rotated_vertices_euler[edge[0]], rotated_vertices_euler[edge[1]]
        line.set_data_3d(*zip(start, end))
    
    # Update Quaternion lines
    for edge, line in zip(edges, lines_quaternion):
        start, end = rotated_vertices_quaternion[edge[0]], rotated_vertices_quaternion[edge[1]]
        line.set_data_3d(*zip(start, end))
    
    # Simulate gimbal lock around 90 degrees pitch for Euler
    if 85 <= pitch % 360 <= 95:
        ax1.set_title("Euler Angles (Gimbal Lock!)")
    else:
        ax1.set_title("Euler Angles (Gimbal Lock)")

# Create the animation
ani = FuncAnimation(
    fig, update, frames=np.arange(0, 180, 2), interval=50, blit=False
)

# Show the animation
plt.tight_layout()
plt.show()
