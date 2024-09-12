import numpy as np
import cv2
import matplotlib.pyplot as plt

# Load the depth map
depth_map_normalized = cv2.imread('depthmap.png', cv2.IMREAD_GRAYSCALE)


cv2.imshow('d', depth_map_normalized)

# Normalize the depth map to a range of 0 to 1
# depth_map_normalized = cv2.normalize(depth_map, None, 0, 1, cv2.NORM_MINMAX)

# Define the laser scan parameters
num_angles = 360  # Number of angles in the laser scan (360 for a full circle)
max_range = 225.0  # Maximum range of the laser (in arbitrary units)
angles = np.linspace(-np.pi / 2, np.pi / 2, num_angles)  # Angle range [-90, 90] degrees

# Initialize an array to store the range measurements
laser_scan = np.full(num_angles, max_range)

# Project the depth map into the laser scan
height, width = depth_map_normalized.shape
line= int(np.round(height/2))
for x in range(width):
    # Calculate the corresponding angle for each column
    angle = (x / width) * np.pi - (np.pi / 2)
    
    # Get the minimum depth value along the column (closest object)
    depth_value = depth_map_normalized[line, x]
    
    print(depth_value)
    
    # Calculate the range (proportional to the depth)
    # range_value = depth_value * max_range
    
    # Find the closest angle in the laser scan array
    angle_index = np.abs(angles - angle).argmin()
    
    # Update the laser scan with the smaller range (closest object)
    laser_scan[angle_index] = depth_value
print(depth_map_normalized)
# Create a polar scatter plot\
print(laser_scan)
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')

# Convert to radians and plot as scatter
ax.scatter(angles, laser_scan, c='blue', s=10)  # c='blue' for point color, s=10 for size

# Set plot details
ax.set_title('Polar Scatter Plot of Simulated Laser Scan', va='bottom')
ax.set_ylim(0, max_range)

# Show the plot
plt.show()
