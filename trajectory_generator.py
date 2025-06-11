import os
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

file_path = os.path.join(os.getcwd(), 'reference_trajectory.csv')
data = []
t0 = 0.0

def rotation_matrix(axis, theta):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    one_minus_cos = 1 - cos_theta

    return np.array([
        [cos_theta + x*x*one_minus_cos,
         x*y*one_minus_cos - z*sin_theta,
         x*z*one_minus_cos + y*sin_theta],
        [y*x*one_minus_cos + z*sin_theta,
         cos_theta + y*y*one_minus_cos,
         y*z*one_minus_cos - x*sin_theta],
        [z*x*one_minus_cos - y*sin_theta,
         z*y*one_minus_cos + x*sin_theta,
         cos_theta + z*z*one_minus_cos]
    ])

time = np.arange(0, 24, 0.1)
rotational_angles = np.ones_like(time) * 0.1 

m0 = np.array([0.0, 0.8, 0.6])
mcurrent = m0.copy()
trajectory = [m0.copy()]

r_vec_0 = np.array([0.0, 1.25, 1.0])
r_vec_0 = r_vec_0 / np.linalg.norm(r_vec_0, 2)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=-15, azim=135, roll=180)
vector_quiver = ax.quiver(0, 0, 0, *mcurrent, color='navy', length=0.8)
trajectory_line, = ax.plot([], [], [], 'b--', lw=1) 


ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

plot_labels_color = 'black'

ax.set_xlabel('X', fontsize=14, fontweight='bold', color=plot_labels_color)
ax.set_ylabel('Y', fontsize=14, fontweight='bold', color=plot_labels_color)
ax.set_zlabel('Z', fontsize=14, fontweight='bold', color=plot_labels_color)
ax.set_title('Magnet Reference Trajectory', fontsize=14, fontweight='bold', color=plot_labels_color)

ax.set_box_aspect([1, 1, 1])

ax.tick_params(axis='both', which='major', labelsize=8, colors=plot_labels_color)
for label in ax.get_xticklabels() + ax.get_yticklabels() + ax.get_zticklabels():
    label.set_fontweight('bold')
    label.set_color(plot_labels_color)

ax.set_box_aspect([1, 1, 1]) 

def update(frame):
    global mcurrent, vector_quiver, trajectory_line, trajectory, r_vec_0, fps, t0, data

    angle = 0.1
    
    # Determine the rotation axis based on the frame
    r_vec_0 = rotation_matrix(np.array([0.0, 0.0, 1.0]), angle) @ r_vec_0
    axis = r_vec_0.copy()

    # Update rotation
    R = rotation_matrix(axis, angle)
    mcurrent = R @ mcurrent
    trajectory.append(mcurrent.copy())

    t0 += 50 * 1e-3

    data.append(
        {
            'time': t0,
            'm_target_x': mcurrent[0],
            'm_target_y': mcurrent[1],
            'm_target_z': mcurrent[2],
        }
    )

    # Update quivers
    vector_quiver.remove()
    vector_quiver = ax.quiver(0, 0, 0, *mcurrent, color='navy', length=0.8, linewidth=2.0)

    traj_array = np.array(trajectory)
    trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
    trajectory_line.set_3d_properties(traj_array[:, 2])
    trajectory_line.set_linewidth(2)
    trajectory_line.set_color('#CC5500')  # Set to burned orange

    return vector_quiver, trajectory_line


ani = FuncAnimation(fig, update, frames=6000, interval=50, blit=False)

animation_filename = "reference_trajectory.mp4"
fps = 1000 / 50 # frames per second = 1000ms / interval_ms


print(f"Saving animation to {animation_filename}. This may take some time...")
ani.save(animation_filename, writer='ffmpeg', fps=fps)
print("Animation saved successfully!")

# plt.show()

df = pd.DataFrame(data)
df.to_csv(file_path, index=False)
print(f"[✓] Log saved to {file_path}")