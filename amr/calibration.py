import numpy as np
import cv2 as cv
from os import listdir, mkdir
from os.path import join, isdir
import matplotlib.pyplot as plt

### Paths
ROOT = r'/home/sg/project'
output_id = '7.jpg'


### Termination Critera, Modes
criteria_calib = (cv.TERM_CRITERIA_MAX_ITER + cv.TERM_CRITERIA_EPS, 1000, 1e-6)
params_ransac = (cv.FM_RANSAC, 2.5, 0.9)
flags_thresh = (cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
flags_indiv_calib = (0)
flags_stereo_calib = (cv.CALIB_FIX_INTRINSIC)


def sort_id(e):
    return int(e.split('.')[0])


def rescaleROI(src, roi):
    x, y, w, h = roi
    dst = src[y:y+h, x:x+w]
    return dst


file_list = [i for i in listdir(join(ROOT, 'L'))]
file_list.sort(key=sort_id)

objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

objPts = []
imgPL, imgPR = [], []

imgSize = (640, 480) # (768, 1024), 1296x972


for f in file_list:
    fname = str(f)

    imgL = cv.imread(join(ROOT, 'L', fname))
    imgR = cv.imread(join(ROOT, 'R', fname))
    grayL = cv.cvtColor(imgL, cv.COLOR_RGB2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_RGB2GRAY)
    # h,w = grayL.shape
    retL, cornersL = cv.findChessboardCorners(grayL, (7,7), flags=flags_thresh)
    retR, cornersR = cv.findChessboardCorners(grayR, (7,7), flags=flags_thresh)

    if retL and retR:
        objPts.append(objp)
        corners2L = cv.cornerSubPix(grayL, cornersL, (3,3), (-1,-1), criteria_calib)
        imgPL.append(corners2L)
        # objPR.append(objp)
        corners2R = cv.cornerSubPix(grayR, cornersR, (3,3), (-1,-1), criteria_calib)
        imgPR.append(corners2R)

    else:
        print(f'Corners not found in {fname} (Left={retL}, Right={retR})')
        break

objPts = np.asarray(objPts, np.float32)
imgPL = np.asarray(imgPL, np.float32)
imgPR = np.asarray(imgPR, np.float32)



mse1, C1, D1, R1, T1 = cv.calibrateCamera(objPts, imgPL, imgSize, 
                        None, None, flags=flags_indiv_calib, criteria=criteria_calib)
mse2, C2, D2, R2, T2 = cv.calibrateCamera(objPts, imgPR, imgSize, 
                        None, None, flags=flags_indiv_calib, criteria=criteria_calib)

mseTotal,CL,DL,CR,DR,R,T,E,F = cv.stereoCalibrate(objPts, imgPL, imgPR, 
                        C1, D1, C2, D2, imgSize, flags=flags_stereo_calib, criteria=criteria_calib)


### Rectification Transforms, Projection Matrices, ROIs after rectification
RL,RR,PL,PR,Q,validROIL,validROIR = cv.stereoRectify(CL, DL, CR, DR, imgSize, R, T, alpha=0, 
                                                     newImageSize=imgSize, flags=cv.CALIB_ZERO_DISPARITY)


''' Print critical parameters and per-view reprojection error '''
np.set_printoptions(suppress=True, precision=3)

print('CameraMatrix_L = \n{}\n\nCameraMatrix_R = \n{}\n'.format(CL, CR))
print('RotationStereo = \n{}\n\nTranslationStereo = \n{}\n'.format(R, T))
print('DistCoeffStereo_L = \n{}\n\nDistCoeffStereo_R = \n{}\n'.format(DL, DR))
print('Q = \n{}\n'.format(Q))

print(f'Left MSE: {mse1:0.6f}')
print(f'Right MSE: {mse1:0.6f}')
print(f'Overall MSE: {mseTotal:0.6f} ({len(file_list)} image pairs)\n')

labels = []



''' Rectification mapping '''
undistL, rectifL = cv.initUndistortRectifyMap(CL, DL, RL, PL, imgSize, cv.CV_32FC1)
undistR, rectifR = cv.initUndistortRectifyMap(CR, DR, RR, PR, imgSize, cv.CV_32FC1)


''' Preview rectification & remap '''
img1 = cv.imread(join(ROOT, 'L', output_id))
img2 = cv.imread(join(ROOT, 'R', output_id))
img1 = cv.cvtColor(img1, cv.COLOR_BGR2RGB)
img2 = cv.cvtColor(img2, cv.COLOR_BGR2RGB)

img1 = cv.remap(img1, undistL, rectifL, cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
img2 = cv.remap(img2, undistR, rectifR, cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

plt.figure(figsize=(9,6))
plt.subplot(221); plt.imshow(img1); plt.title('remap_L: ' + str(img1.shape))
plt.subplot(222); plt.imshow(img2); plt.title('remap_R: ' + str(img2.shape))

img11 = rescaleROI(img1, validROIL)
img22 = rescaleROI(img2, validROIR)

# dsize = (img11.shape[1], img11.shape[0])
# img22 = cv.resize(img22, dsize, interpolation=cv.INTER_LINEAR)

### Compare rectified and remapped images
plt.subplot(223); plt.imshow(img11); plt.title('ROI_L: ' + str(img11.shape))
plt.subplot(224); plt.imshow(img22); plt.title('ROI_R: ' + str(img22.shape))
plt.tight_layout()
plt.show()


''' Write parameters to .txt files '''
output_dir = r'Calibration_Files_expm'
prompt = input('Save parameters to "{}\\"? (y/n): '.format(output_dir))

if (prompt == 'y'):
    if not isdir(output_dir):
        mkdir(output_dir)

    np.savetxt(join(output_dir, 'Q.txt'), Q, fmt='%.5e')
    np.savetxt(join(output_dir, 'CmL.txt'), CL, fmt='%.5e')
    np.savetxt(join(output_dir, 'CmR.txt'), CR, fmt='%.5e')
    np.savetxt(join(output_dir, 'DcL.txt'), DL, fmt='%.5e')
    np.savetxt(join(output_dir, 'DcR.txt'), DR, fmt='%.5e')
    np.savetxt(join(output_dir, 'Rtn.txt'), R, fmt='%.5e')
    np.savetxt(join(output_dir, 'Trnsl.txt'), T, fmt='%.5e')
    np.savetxt(join(output_dir, 'RectifL.txt'), RL, fmt='%.5e')
    np.savetxt(join(output_dir, 'ProjL.txt'), PL, fmt='%.5e')
    np.savetxt(join(output_dir, 'ProjR.txt'), PR, fmt='%.5e')
    np.savetxt(join(output_dir, 'umapL.txt'), undistL, fmt='%.5e')
    np.savetxt(join(output_dir, 'rmapL.txt'), rectifL, fmt='%.5e')
    np.savetxt(join(output_dir, 'umapR.txt'), undistR, fmt='%.5e')
    np.savetxt(join(output_dir, 'rmapR.txt'), rectifR, fmt='%.5e')
    np.savetxt(join(output_dir, 'ROIL.txt'), validROIL, fmt='%.5e')
    np.savetxt(join(output_dir, 'ROIR.txt'), validROIR, fmt='%.5e')
    print(f'Parameters saved to folder: [{output_dir}]')
