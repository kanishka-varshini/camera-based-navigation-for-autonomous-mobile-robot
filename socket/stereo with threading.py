import threading
import time
import numpy as np
import cv2 as cv
import pickle
import socket
import struct
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import time
from picamera2 import Picamera2
from cv2 import ximgproc
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from os.path import isfile, join


# Camera setup
picam2L = Picamera2(camera_num=0)
picam2R = Picamera2(camera_num=1)
video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
picam2L.configure(video_config)
picam2R.configure(video_config)
picam2L.start()
picam2R.start()




'''Global Variables '''

'''
PATH_L = r'/home/sg/project/L'
PATH_R = r'/home/sg/project/R'
'''

PATH_CALIB = r'/home/sg/project/Calibration_Files_expm'
useStream = 1

### Stereo Matcher Parameters
minDisp = 0     # window position x-offset
nDisp = 16*1      # Range of visible depths, larger num takes longer (./16)
bSize = 7      # Size of search windows, (typ. 7-15, odd # only)
# P1 = 8*3*bSize**2
# P2 = 32*3*bSize**2
# modeSgbm = cv.StereoSGBM_MODE_SGBM   # Default: cv.StereoSGBM_MODE_SGBM
# pfCap = 0
# sRange = 0
yfloor = 340     # y-axis pixel location of floor plane (scene-specific)

### Weighted least squares parameters
lam = 32000    # Regularization param
sigma = 2.5      # Contrast sensitivity param
discontinuityRad = 4

#params = [minDisp, nDisp, bSize, pfCap, sRange]
params = [minDisp, nDisp, bSize]


### Load Camera Calibration Parameters
undistL = np.loadtxt(join(PATH_CALIB, 'umapL.txt'), dtype=np.float32)
rectifL = np.loadtxt(join(PATH_CALIB, 'rmapL.txt'), dtype=np.float32)
undistR = np.loadtxt(join(PATH_CALIB, 'umapR.txt'), dtype=np.float32)
rectifR = np.loadtxt(join(PATH_CALIB, 'rmapR.txt'), dtype=np.float32)
roiL = np.loadtxt(join(PATH_CALIB, 'ROIL.txt'), dtype=int)
roiR = np.loadtxt(join(PATH_CALIB, 'ROIR.txt'), dtype=int)
Q = np.loadtxt(join(PATH_CALIB, 'Q.txt'), dtype=np.float32)
#R = np.loadtxt(join(PATH_CALIB, 'Rtn.txt'), dtype=np.float32)
#T = np.loadtxt(join(PATH_CALIB, 'Trnsl.txt'), dtype=np.float32)
#PL = np.loadtxt(join(PATH_CALIB, 'ProjL.txt'), dtype=np.float32)
#PR = np.loadtxt(join(PATH_CALIB, 'ProjR.txt'), dtype=np.float32)
RL = np.loadtxt(join(PATH_CALIB, 'RectifL.txt'), dtype=np.float32)
CL = np.loadtxt(join(PATH_CALIB, 'CmL.txt'), dtype=np.float32)
DL = np.loadtxt(join(PATH_CALIB, 'DcL.txt'), dtype=np.float32)

''' End Global Variables '''


# Socket setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laptop_ip = '10.7.30.173'
laptop_port = 8080


# Connect to laptop
client_socket.connect((laptop_ip, laptop_port))

# Global variables
running = True

def capture_images():
    global running
    while running:
        # Capture frames from both cameras
        imgL = picam2L.capture_array()
        imgR = picam2R.capture_array()

        # Send images to laptop for SLAM processing
        send_data(imgL, imgR)

        # Wait for the next frame
        time.sleep(0.1)

def send_data(imgL, imgR):
    try:
        # Encode images as JPEG to reduce size
        _, bufferL = cv.imencode('.jpg', imgL)
        _, bufferR = cv.imencode('.jpg', imgR)

        # Serialize images and send
        data = {'imgL': bufferL.tobytes(), 'imgR': bufferR.tobytes()}
        serialized_data = pickle.dumps(data)
        data_size = struct.pack('>I', len(serialized_data))

        # Send data size and then the serialized images
        client_socket.sendall(data_size)
        client_socket.sendall(serialized_data)

    except Exception as e:
        print(f"Error in send_data: {e}")

# def process_disparity(imgL, imgR):
#     # Convert images to grayscale
#     grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
#     grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

#     # Stereo Matching (using StereoBM for block matching)
#     stereo = cv.StereoBM_create(numDisparities=16, blockSize=7)
#     dispL = stereo.compute(grayL, grayR)

#     # Apply WLS filter for disparity refinement
#     # Create the disparity object for the right image
#     stereo_right = cv.StereoBM_create(numDisparities=16, blockSize=7)
#     dispR = stereo_right.compute(grayR, grayL)

#     # Create WLS filter and apply it
#     wls_filter = cv.ximgproc.createDisparityWLSFilter(stereo)
#     filtered_disp = wls_filter.filter(dispL, grayL, disparity_map_right=dispR)

#     # Normalize disparity for display (original style with color grading)
#     dispFinal = cv.normalize(filtered_disp, None, 0, 255, cv.NORM_MINMAX)
#     dispFinal = np.uint8(dispFinal)

#     # Apply color map (similar to original effect with color grading)
#     dispColor = cv.applyColorMap(dispFinal, cv.COLORMAP_JET)

#     # Display disparity map
#     cv.imshow('Disparity Map with WLS', dispColor)





def rescaleROI(src, roi):
    x, y, w, h = roi
    dst = src[y:y+h, x:x+w]
    return dst


def compute_disparity(imgL, imgR, params):
    # Convert images from RGB to BGR if necessary
    imgL = cv.cvtColor(imgL, cv.COLOR_RGB2BGR)
    imgR = cv.cvtColor(imgR, cv.COLOR_RGB2BGR)

    imgL = cv.remap(imgL, undistL, rectifL, cv.INTER_LINEAR)
    imgR = cv.remap(imgR, undistR, rectifR, cv.INTER_LINEAR)

    imgL = rescaleROI(imgL, roiL)
    imgR = rescaleROI(imgR, roiR)

    if imgL.shape != imgR.shape:
        print('L.shape != R.shape: {} != {}'.format(imgL.shape, imgR.shape))
        dsize = (imgL.shape[1], imgL.shape[0])
        imgR = cv.resize(imgR, dsize, interpolation=cv.INTER_LINEAR)

    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    ### Init StereoMatcher with parameters
    # (minDisp, nDisp, bSize, pfCap, sRange) = params
    # stereoL = cv.StereoSGBM_create(
    #     minDisparity=minDisp,
    #     numDisparities=nDisp,
    #     blockSize=bSize,
    #     P1=P1,
    #     P2=P2,
    #     speckleRange=sRange,
    #     preFilterCap=pfCap,
    #     mode=modeSgbm)

    stereoL = cv.StereoBM_create(numDisparities=nDisp, blockSize=bSize)


    ### Init WLS Filter with parameters
    wls = ximgproc.createDisparityWLSFilter(stereoL)
    stereoR = ximgproc.createRightMatcher(stereoL)
    wls.setLambda(lam)
    wls.setDepthDiscontinuityRadius(discontinuityRad)  # Default 4
    wls.setSigmaColor(sigma)

    ### Compute raw disparity from both sides
    ts1 = time.time()
    dispL = stereoL.compute(grayL, grayR)
    dispR = stereoR.compute(grayR, grayL)
    ts2 = time.time()
    cost_sgbm = ts2 - ts1

    ### Filter raw disparity using weighted least squares based smoothing
    dispFinal = wls.filter(dispL, imgL, None, dispR)
    dispFinal = ximgproc.getDisparityVis(dispFinal)

    paramsVals = [sigma, lam,
                  stereoL.getNumDisparities(), stereoL.getBlockSize(),
                  stereoL.getPreFilterCap(), stereoL.getSpeckleRange()]

    ''' Map disparity values to depth as 3D point cloud '''
    points3d = cv.reprojectImageTo3D(
        dispFinal, Q, ddepth=cv.CV_32F, handleMissingValues=True)

    ''' Filter obstacles, compute occupancy grid, find path '''
    find_path(imgL, nDisp, points3d, dispFinal, cost_sgbm)



def find_path(imgL, nDisp, points3d, disparityMap, cost_sgbm):
    np.set_printoptions(suppress=True, precision=3)
    xx, yy, zz = points3d[:,:,0], points3d[:,:,1], points3d[:,:,2]
    xx, yy, zz = np.clip(xx, -25, 60), np.clip(yy, -25, 25), np.clip(zz, 0, 100)

    ''' Filter obstacles above ground/floor plane '''
    obs = zz[yfloor-10:yfloor,:]

    ''' Construct occupancy grid '''
    obstacles = np.amin(obs, 0, keepdims=False)
    y = np.mgrid[0:np.amax(obstacles), 0:obs.shape[1]][0,:,:]

    ### Assign weights to regions (cost low -> high == 0.01 -> 2)
    occupancy_grid = np.where(y >= obstacles, 0, 1)
    occupancy_grid[:, :nDisp+60] = 0

    far_zy, far_zx = np.unravel_index(np.argmax(np.flip(occupancy_grid[:,:-90])), occupancy_grid[:,:-90].shape)
    far_zx = (zz.shape[1]-91) - far_zx

    far_zy = occupancy_grid.shape[0] - far_zy - 1

    xcenter = 305

    ''' A* path-finding config and computation '''
    mat_grid = Grid(matrix=occupancy_grid)
    start = mat_grid.node(xcenter, 1)
    end = mat_grid.node(far_zx, far_zy)
    tp1 = time.time()
    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start, end, mat_grid)
    tp2 = time.time()
    cost_path = tp2 - tp1

    if len(path) == 0:
        print('ERROR: No path found')

    ''' Map X,Y pixel positions to world-frame for cv.projectPoints() '''
    coords = np.array([(xp, zp) for xp, zp in path], dtype=np.int32)

    yrange = np.geomspace(yy.shape[0]-1, yfloor+1, num=len(path), dtype=np.int32)
    yrange = np.flip(yrange)

    yworld = np.geomspace(10,13, num=len(path), dtype=np.float32)
    xworld = xx[yrange, coords[:,0]]
    zworld = np.array([zp for _, zp in path], dtype=np.float32)
    zworld = np.interp(zworld, [0, np.amax(zworld)], [25, nDisp])

    cf = np.array([xworld, yworld, zworld]).T

    ''' Reproject 3D world-frame points back to unrectified 2D points'''
    pr, _ = cv.projectPoints(cf, np.zeros(3), np.zeros(3), CL, DL)
    pr = np.squeeze(pr, 1)
    py = pr[:,1]
    px = pr[:,0]

    ''' Draw Floor Polygon '''
    fPts = np.array([[-40, 13, nDisp], [40, 13, nDisp], [40, 15, 0], [-40, 15, 0]], dtype=np.float32).T
    pf, _ = cv.projectPoints(fPts, np.zeros(3).T, np.zeros(3), CL, None)
    pf = np.squeeze(pf, 1)

    ''' Update figure (final results) '''
    # Convert imgL from BGR to RGB for plotting
    imL = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)

    plt.clf()
    plt.suptitle('Live Feed')

    costStats = '(far_zx, far_zy)=({},{})\ncost_path={:.3f}\ncost_sgbm={:.3f}'.format(
        far_zx, far_zy, cost_path, cost_sgbm)
    plt.gcf().text(x=0.6, y=0.05, s=costStats, fontsize='small')

    pathStats = 'steps={}\npathlen={}'.format(runs, len(path))
    plt.gcf().text(x=0.75, y=0.05, s=pathStats, fontsize='small')

    plt.subplot(221)
    plt.imshow(imL)
    plt.title('Planned Path (Left Camera)')
    plt.xlim([0, 640])
    plt.ylim([480, 0])
    plt.scatter(px, py, s=np.geomspace(70, 5, len(px)), c=cf[:,1],
                cmap=plt.cm.plasma_r, zorder=99)
    plt.gca().add_patch(Polygon(pf, fill=True, facecolor=(0,1,0,0.12),
                                edgecolor=(0,1,0,0.35)))

    ax = plt.gcf().add_subplot(222, projection='3d')
    ax.azim = 90
    ax.elev = 110
    ax.set_box_aspect((4,3,3))
    ax.plot_surface(xx[100:yfloor,:], yy[100:yfloor,:], zz[100:yfloor,:],
                    cmap=plt.cm.viridis_r, rcount=25, ccount=25, linewidth=0,
                    antialiased=False)
    ax.set_xlabel('Azimuth (X)')
    ax.set_ylabel('Elevation (Y)')
    ax.set_zlabel('Depth (Z)')
    ax.invert_xaxis()
    ax.invert_zaxis()
    ax.set_title('Planned Path (wrt. world-frame)')
    ax.scatter3D(cf[:,0], cf[:,1], cf[:,2], c=cf[:,2], cmap=plt.cm.plasma_r)

    plt.subplot(223)
    plt.imshow(disparityMap)
    plt.title('WLS Filtered Disparity Map')

    plt.subplot(224)
    plt.imshow(occupancy_grid, origin='lower', interpolation='none')
    plt.title('Occupancy Grid with A* Path')
    plt.plot(coords[:,0], coords[:,1], 'r')  # Plot A* path over occupancy grid


def display_disparity(origImg, dispRaw, dispWLS, imgName, paramsVals):
    ''' Helper function to show figure with some parameters '''

    # Show parameters on figure for debugging
    paramsText = 'Lambda={}; Sigma={}; nDisp={}; bSize={}; pfCap={}; sRange={}'.format(*paramsVals)

    fig, (ax1, ax2, ax3) = plt.subplots(figsize=(12, 4), ncols=3)
    #fig.subplots_adjust(hspace=0.3)
    plt.suptitle(imgName)
    plt.gcf().text(x=0.1, y=0.05, s=paramsText)

    origImg = cv.cvtColor(origImg, cv.COLOR_BGR2RGB)

    ax1.imshow(origImg, interpolation='none'); ax1.set_title('Original (Left Camera)')
    pl2 = ax2.imshow(dispRaw, 'gray', interpolation='none'); ax2.set_title('Raw Disparity')
    pl3 = ax3.imshow(dispWLS, interpolation='none'); ax3.set_title('WLS Filter')
    
    cbar2 = fig.colorbar(pl2, ax=ax2, fraction=0.034, pad=0.04)
    cbar2.minorticks_on()
    cbar3 = fig.colorbar(pl3, ax=ax3, fraction=0.034, pad=0.04)
    cbar3.minorticks_on()
    plt.tight_layout()
    plt.show()


#original code for a* ends


def main():
    # Start image capture in a separate thread
    capture_thread = threading.Thread(target=capture_images)
    capture_thread.start()

    try:
        while True:
            # Capture images from both cameras
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()

            # Display the live feed
            cv.imshow('Left Camera Feed', imgL)
            cv.imshow('Right Camera Feed', imgR)

            # Process disparity map with WLS filter
            compute_disparity(imgL, imgR, params)

            # Wait for a key press and handle quitting
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        # Stop the threads gracefully
        global running
        running = False
        capture_thread.join()

    finally:
        picam2L.stop()
        picam2R.stop()
        client_socket.close()
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()
