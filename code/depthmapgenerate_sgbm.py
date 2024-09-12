import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load stereo images (left and right)
l= cv2.imread('left.png', cv2.IMREAD_GRAYSCALE)
r= cv2.imread('right.png', cv2.IMREAD_GRAYSCALE)

scale_ratio = 1

# Get the new dimensions
width = int(l.shape[1] * scale_ratio)
height = int(r.shape[0] * scale_ratio)
dim = (width, height)

# Resize the image
left_image  = cv2.GaussianBlur(cv2.resize(l, dim, interpolation=cv2.INTER_AREA), (3,3), 0)
right_image = cv2.GaussianBlur(cv2.resize(r, dim, interpolation=cv2.INTER_AREA), (3,3), 0)

# left_image  = cv2.resize(l, dim, interpolation=cv2.INTER_AREA)
# right_image = cv2.resize(r, dim, interpolation=cv2.INTER_AREA)

# left_image = cv2.Canny(left_image, 100, 200)
# right_image = cv2.Canny(right_image, 100, 200)

cv2.imshow('l', left_image)
cv2.imshow('r', right_image)

# Set up the initial parameters
initial_focal_length = 3407  # Initial focal length in pixels
initial_baseline = 81  # Initial baseline in mm
num_disparities = 5  # Number of disparities (must be divisible by 16)
block_size = 5  # Block size for stereo matching (must be odd)
p1 = 21
p2 = 30

# Function to compute disparity and depth maps
def compute_maps(focal_length, baseline, num, blocksize, p1, p2):
    # Create stereo block matcher object
    stereo = cv2.StereoSGBM_create(numDisparities=num, blockSize=blocksize, P1 = p1, P2 = p2)
    
    # Compute disparity map
    disparity = stereo.compute(left_image, right_image).astype(np.float32) / 16.0

    # for i in disparity:
    #     print(i)

    disparity[disparity <= 0] = 0
    
    # Compute depth map: depth = (focal_length * baseline) / disparity
    depth_map = (focal_length * baseline) / (disparity)  # Add small value to avoid division by zero
    # for i in depth_map:
    #     print(i)

    depth_map[depth_map>100000] = 100000

    
    # Normalize for visualization
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)
    
    depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    
    depth_vis = np.uint8(depth_vis)
    colored_depth_map = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    
    return disp_vis, colored_depth_map

# Callback function for trackbar changes
def on_trackbar_change(val):
    focal_length = cv2.getTrackbarPos('Focal Length', 'Depth Map')
    baseline = cv2.getTrackbarPos('Baseline', 'Depth Map')

    num_disparities = 16*cv2.getTrackbarPos('num_disparities', 'Disparity Map')
    block_size = 2*cv2.getTrackbarPos('block_size', 'Disparity Map')+1

    p1 = cv2.getTrackbarPos('P1', 'Disparity Map')
    p2 = cv2.getTrackbarPos('P2', 'Disparity Map')
    
    # Compute and display updated maps
    disp_vis, depth_vis = compute_maps(focal_length, baseline, num_disparities, block_size, p1, p2)
    cv2.imshow('Disparity Map', disp_vis)
    cv2.imwrite('depthmap.png', depth_vis)
    cv2.imshow('Depth Map', depth_vis)

# Create windows to display results
cv2.namedWindow('Disparity Map')
cv2.namedWindow('Depth Map')

# Create trackbars for focal length and baseline
cv2.createTrackbar('Focal Length', 'Depth Map', initial_focal_length, 8000, on_trackbar_change)
cv2.createTrackbar('Baseline', 'Depth Map', initial_baseline, 2000, on_trackbar_change)

cv2.createTrackbar('num_disparities', 'Disparity Map', num_disparities, 100, on_trackbar_change)
cv2.createTrackbar('block_size', 'Disparity Map', block_size, 25, on_trackbar_change)
cv2.createTrackbar('P1', 'Disparity Map', p1, 1000, on_trackbar_change)
cv2.createTrackbar('P2', 'Disparity Map', p2, 1000, on_trackbar_change)

# Initialize with the default parameters
on_trackbar_change(0)

# Wait for user input to exit
k = cv2.waitKey(0)
if(k == ord('c')):
    cv2.destroyAllWindows()
