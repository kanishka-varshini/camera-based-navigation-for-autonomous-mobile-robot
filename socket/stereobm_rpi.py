import threading
from os.path import isfile, join
import numpy as np
import cv2 as cv
from cv2 import ximgproc
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import time
from picamera2 import Picamera2

# setting up communication between pi and laptop for SLAM
import socket
import pickle
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laptop_ip = '10.7.26.244'  # laptop's IP address 
laptop_port = 8080  #change

PATH_CALIB = r'/home/sg/project/Calibration_Files_expm'
useStream = 1

# Stereo Matcher Parameters
minDisp = 0     
nDisp = 50      
bSize = 9      
yfloor = 340     

# Weighted least squares parameters
lam = 32000    
sigma = 2.5      
discontinuityRad = 4

params = [minDisp, nDisp, bSize]

# Load Camera Calibration Parameters
undistL = np.loadtxt(join(PATH_CALIB, 'umapL.txt'), dtype=np.float32)
rectifL = np.loadtxt(join(PATH_CALIB, 'rmapL.txt'), dtype=np.float32)
undistR = np.loadtxt(join(PATH_CALIB, 'umapR.txt'), dtype=np.float32)
rectifR = np.loadtxt(join(PATH_CALIB, 'rmapR.txt'), dtype=np.float32)
roiL = np.loadtxt(join(PATH_CALIB, 'ROIL.txt'), dtype=int)
roiR = np.loadtxt(join(PATH_CALIB, 'ROIR.txt'), dtype=int)
Q = np.loadtxt(join(PATH_CALIB, 'Q.txt'), dtype=np.float32)
RL = np.loadtxt(join(PATH_CALIB, 'RectifL.txt'), dtype=np.float32)
CL = np.loadtxt(join(PATH_CALIB, 'CmL.txt'), dtype=np.float32)
DL = np.loadtxt(join(PATH_CALIB, 'DcL.txt'), dtype=np.float32)

def capture_frames(picam2L, picam2R):
    while True:
        imgL = picam2L.capture_array()
        imgR = picam2R.capture_array()
        send_data(imgL, imgR)
        compute_disparity(imgL, imgR, params)
        time.sleep(0.2)

def send_data(imgL, imgR):
    try:
        # Encode the images as JPEG to reduce the size
        _, bufferL = cv.imencode('.jpg', imgL)
        _, bufferR = cv.imencode('.jpg', imgR)

        # Serialize the images with pickle
        data = {'imgL': bufferL.tobytes(), 'imgR': bufferR.tobytes()}
        serialized_data = pickle.dumps(data)

        # Send the size of the serialized data first (4 bytes for size)
        data_size = struct.pack('>I', len(serialized_data))
        client_socket.sendall(data_size)

        # Now send the actual serialized data
        client_socket.sendall(serialized_data)
    except Exception as e:
        print(f"An error occurred while sending data to laptop: {e}")

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

    stereoL = cv.StereoBM_create(numDisparities=nDisp, blockSize=bSize)

    wls = ximgproc.createDisparityWLSFilter(stereoL)
    stereoR = ximgproc.createRightMatcher(stereoL)
    wls.setLambda(lam)
    wls.setDepthDiscontinuityRadius(discontinuityRad)
    wls.setSigmaColor(sigma)

    ts1 = time.time()
    dispL = stereoL.compute(grayL, grayR)
    dispR = stereoR.compute(grayR, grayL)
    ts2 = time.time()
    cost_sgbm = ts2 - ts1

    dispFinal = wls.filter(dispL, imgL, None, dispR)
    dispFinal = ximgproc.getDisparityVis(dispFinal)

    paramsVals = [sigma, lam, stereoL.getNumDisparities(), stereoL.getBlockSize(), stereoL.getPreFilterCap(), stereoL.getSpeckleRange()]

    points3d = cv.reprojectImageTo3D(dispFinal, Q, ddepth=cv.CV_32F, handleMissingValues=True)

    find_path(imgL, nDisp, points3d, dispFinal, cost_sgbm)

def find_path(imgL, nDisp, points3d, disparityMap, cost_sgbm):
    np.set_printoptions(suppress=True, precision=3)
    xx, yy, zz = points3d[:,:,0], points3d[:,:,1], points3d[:,:,2]
    xx, yy, zz = np.clip(xx, -25, 60), np.clip(yy, -25, 25), np.clip(zz, 0, 100)

    obs = zz[yfloor-10:yfloor,:]
    obstacles = np.amin(obs, 0, keepdims=False)
    y = np.mgrid[0:np.amax(obstacles), 0:obs.shape[1]][0,:,:]

    occupancy_grid = np.where(y >= obstacles, 0, 1)
    occupancy_grid[:, :nDisp+60] = 0

    far_zy, far_zx = np.unravel_index(np.argmax(np.flip(occupancy_grid[:,:-90])), occupancy_grid[:,:-90].shape)
    far_zx = (zz.shape[1]-91) - far_zx

    far_zy = occupancy_grid.shape[0] - far_zy - 1

    xcenter = 305

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

    coords = np.array([(xp, zp) for xp, zp in path], dtype=np.int32)
    yrange = np.geomspace(yy.shape[0]-1, yfloor+1, num=len(path), dtype=np.int32)
    yrange = np.flip(yrange)

    yworld = np.geomspace(10,13, num=len(path), dtype=np.float32)
    xworld = xx[yrange, coords[:,0]]
    zworld = np.array([zp for _, zp in path], dtype=np.float32)
    zworld = np.interp(zworld, [0, np.amax(zworld)], [25, nDisp])

    cf = np.array([xworld, yworld, zworld]).T

    pr, _ = cv.projectPoints(cf, np.zeros(3), np.zeros(3), CL, DL)
    pr = np.squeeze(pr, 1)
    py = pr[:,1]
    px = pr[:,0]

    fPts = np.array([[-40, 13, nDisp], [40, 13, nDisp], [40, 15, 0], [-40, 15, 0]], dtype=np.float32).T
    pf, _ = cv.projectPoints(fPts, np.zeros(3).T, np.zeros(3), CL, None)
    pf = np.squeeze(pf, 1)

    imL = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)

    plt.clf()
    plt.suptitle('Live Feed')

    costStats = '(far_zx, far_zy)=({},{})\ncost_path={:.3f}\ncost_sgbm={:.3f}'.format(far_zx, far_zy, cost_path, cost_sgbm)
    plt.gcf().text(x=0.6, y=0.05, s=costStats, fontsize='small')

    pathStats = 'steps={}\npathlen={}'.format(runs, len(path))
    plt.gcf().text(x=0.6, y=0.025, s=pathStats, fontsize='small')

    if len(coords) > 0:
        plt.plot(px, py, marker='.', markersize=2, label='Path', color='g')
    else:
        plt.plot([0], [0], marker='.', markersize=1, color='r')

    plt.plot(pf[:, 0], pf[:, 1], marker='.', markersize=4, label='Far obstacles', color='r')

    plt.imshow(imL)
    plt.pause(0.05)
    plt.show()

if __name__ == "__main__":
    picam2L = Picamera2()
    picam2R = Picamera2()

    picam2L.configure(picam2L.create_still_configuration())
    picam2R.configure(picam2R.create_still_configuration())

    picam2L.start()
    picam2R.start()

    capture_thread = threading.Thread(target=capture_frames, args=(picam2L, picam2R))
    capture_thread.start()
