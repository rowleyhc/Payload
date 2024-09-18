import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import pandas as pd
import time
import numpy as np

# Load CSV file
df = pd.read_csv('Simulations & Team Tools/cleaned_flight_data/TADPOL_April_Cleaned.csv')


def is_valid_quaternion(q):
    return np.linalg.norm(q) > 0

rotation_matrices = []
times = []
stages = []
for _, row in df.iterrows():
    quat = [row['IMU OrientationX'], row['IMU OrientationY'], row['IMU OrientationZ'], row['IMU OrientationW']]
    
    # Check if the quaternion is valid
    if is_valid_quaternion(quat):
        rotation_matrices.append(R.from_quat(quat).as_matrix())
        times.append(row['Time (ms)'])
        stages.append(row['Stage'])
    else:
        continue

# Downsize the matrices by n
n = 3
rotation_matrices = rotation_matrices[::n]
times = times[::n]
stages = stages[::n]

# Initialize the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create a block (or arrow) vertices
block_vertices = np.array([
    [-1, -2, -1], [1, -2, -1], [1, 2, -1], [-1, 2, -1],
    [-1, -2, 1], [1, -2, 1], [1, 2, 1], [-1, 2, 1]
])

# Define the 12 triangles composing the block
block_faces = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [4, 7, 3, 0],
]

# Create and initialize the block plot elements
poly3d = Poly3DCollection([], alpha=.25, linewidths=1, edgecolors='r')
ax.add_collection3d(poly3d)

# Set the plot limits and title
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_title("3D Block Animation")

# Create a slider axis and the slider widget
ax_slider = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(ax_slider, 'Frame', 0, len(rotation_matrices) - 1, valinit=0, valfmt='%0.0f')

# Track whether the slider is being used
slider_active = [False]

# Function to update the plot
def update(frame):
    if not slider_active[0]:  # Update only if slider is not being actively adjusted
        slider.set_val(frame)  # Sync slider value
    frame = int(slider.val)
    rot_matrix = rotation_matrices[frame]
    rotated_vertices = np.dot(block_vertices, rot_matrix.T)
    
    faces_rotated = [[rotated_vertices[vertex] for vertex in face] for face in block_faces]
    poly3d.set_verts(faces_rotated)

    # Update the title with the current time
    current_time = times[frame]
    current_stage = stages[frame]
    ax.set_title(f"Time: {current_time} ms   Stage: {current_stage}")
    
    return poly3d,


ani = FuncAnimation(
    fig, update, frames=len(rotation_matrices),
    interval=20,  # Update interval for animation
    repeat=True
)

def slider_update(val):
    slider_active[0] = True
    frame = int(slider.val)
    update(frame)
    fig.canvas.draw_idle()
    slider_active[0] = False

slider.on_changed(slider_update)

plt.show()