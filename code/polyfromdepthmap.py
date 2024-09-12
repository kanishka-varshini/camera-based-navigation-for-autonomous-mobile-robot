import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the depth map
depth_map_normalized = cv2.imread('depthmap.png', cv2.IMREAD_GRAYSCALE)

# Normalize the depth map to a range of 0 to 1

# Get the height and width of the depth map
height, width = depth_map_normalized.shape

# Create a meshgrid of (x, y) coordinates
x = np.linspace(0, width - 1, width)
y = np.linspace(0, height - 1, height)
x, y = np.meshgrid(x, y)

# Flatten the arrays for plotting
x_flat = x.flatten()
y_flat = y.flatten()
z_flat = depth_map_normalized.flatten()

# Save to .ply file
def save_ply(x, y, z, filename):
    with open(filename, 'w') as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {len(x)}\n")
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("end_header\n")
        for xi, yi, zi in zip(x, y, z):
            file.write(f"{xi} {yi} {zi}\n")

save_ply(x_flat, y_flat, z_flat, 'depthmap.ply')

# Optional: To verify, you can load the .ply file using an external viewer or library

# To plot and save a visual representation of the 3D scatter plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_flat, y_flat, z_flat, c=z_flat, cmap='viridis', marker='.')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Depth (Z)')
ax.set_title('3D Scatter Plot from Depth Map')
plt.show()
