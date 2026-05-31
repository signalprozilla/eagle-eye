# import pandas as pd
# import matplotlib.pyplot as plt

# # Load the data you just generated
# df = pd.read_csv('../build/elbow_trajectory.csv')

# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')

# # Plot the X, Y, Z path of the elbow
# ax.plot(df['x'], df['y'], df['z'], label='Medial Elbow (RMELB)', color='blue')

# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
# ax.set_title('Eagle Eye: 3D Elbow Trajectory')
# plt.legend()
# plt.show()

### new code may 31st:

import numpy as np
import matplotlib.pyplot as plt
from ezc3d import c3d

# 1. Target the exact file that successfully processed in your C++ engine
c3d_path = "../data/obp_repo/baseball_pitching/data/c3d/000002/000002_003034_73_207_002_FF_809.c3d"

print(f"Opening C3D file directly via Python: {c3d_path}")
reader = c3d(c3d_path)

# 2. Extract marker labels and locate the Medial Elbow (RMELB) index
labels = reader["parameters"]["POINT"]["LABELS"]["value"]
marker_index = -1

for i, label in enumerate(labels):
    if "RMELB" in label.upper() or "ELB" in label.upper():
        marker_index = i
        print(f"Found elbow marker '{label}' at data index {i}")
        break

if marker_index == -1:
    # Fallback to index 0 if exact label string differs slightly
    marker_index = 0
    print(f"Warning: Exact 'RMELB' label not found. Defaulting to marker index 0 ({labels[0]})")

# 3. Pull the 3D X, Y, Z trajectory data across all frames
# ezc3d shape is [coordinates (xyz), markers, frames]
data = reader["data"]["points"]
x_coords = data[0, marker_index, :]
y_coords = data[1, marker_index, :]
z_coords = data[2, marker_index, :]

# Convert millimeters to meters if the dataset uses mm scales
if np.max(np.abs(x_coords)) > 10.0:
    x_coords /= 1000.0
    y_coords /= 1000.0
    z_coords /= 1000.0

# 4. Generate the 3D Spatial Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x_coords, y_coords, z_coords, label=f'Trajectory: {labels[marker_index]}', color='blue', lw=2)
ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='green', s=50, label='Pitch Start')
ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='red', s=50, label='Pitch End')

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('Eagle Eye Phase 1: 3D Elbow Kinematic Flight Path')
ax.grid(True)
plt.legend()

print("Rendering 3D spatial plot window...")
plt.show()