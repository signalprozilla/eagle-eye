import pandas as pd
import matplotlib.pyplot as plt

# Load the data you just generated
df = pd.read_csv('../build/elbow_trajectory.csv')

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the X, Y, Z path of the elbow
ax.plot(df['x'], df['y'], df['z'], label='Medial Elbow (RMELB)', color='blue')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Eagle Eye: 3D Elbow Trajectory')
plt.legend()
plt.show()