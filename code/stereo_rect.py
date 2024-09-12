import cv2
import numpy as np

# Load the stereo images
left_image = cv2.imread('l2.jpg', cv2.IMREAD_GRAYSCALE)
right_image = cv2.imread('r2.jpg', cv2.IMREAD_GRAYSCALE)

# Load or define the camera matrices (intrinsic parameters)
camera_matrix_left = np.array([[3406.8, 0, 2312],
                          [0, 3406.8, 1040],
                          [0, 0, 1]])
camera_matrix_right = np.array([[3406.8, 0, 2312],
                          [0, 3406.8, 1040],
                          [0, 0, 1]])

# Load or define the distortion coefficients (if any)
dist_coeffs_left = np.zeros((5, 1))  # Assuming no distortion
dist_coeffs_right = np.zeros((5, 1))

# Rotation (R) and Translation (T) between the two cameras
R = np.eye(3)  # Assuming no rotation between cameras for simplicity
T = np.array([[82.5], [0], [0]])  # Example translation, 120 mm baseline

# Image size (height, width)
image_size = (left_image.shape[1], left_image.shape[0])

# Compute the rectification transformations
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(camera_matrix_left, dist_coeffs_left,
                                                   camera_matrix_right, dist_coeffs_right,
                                                   image_size, R, T, alpha=0)

# Compute the rectification maps for each camera
left_map_x, left_map_y = cv2.initUndistortRectifyMap(camera_matrix_left, dist_coeffs_left, R1, P1, image_size, cv2.CV_32FC1)
right_map_x, right_map_y = cv2.initUndistortRectifyMap(camera_matrix_right, dist_coeffs_right, R2, P2, image_size, cv2.CV_32FC1)

# Apply the rectification maps to the images
rectified_left = cv2.remap(left_image, left_map_x, left_map_y, cv2.INTER_LINEAR)
rectified_right = cv2.remap(right_image, right_map_x, right_map_y, cv2.INTER_LINEAR)

# Save or display the rectified images
cv2.imshow('Rectified Left Image', rectified_left)
cv2.imshow('Rectified Right Image', rectified_right)
cv2.imwrite('rectified_left_image.png', rectified_left)
cv2.imwrite('rectified_right_image.png', rectified_right)

cv2.waitKey(0)
cv2.destroyAllWindows()
