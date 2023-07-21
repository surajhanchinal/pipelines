import cv2
import numpy as np
from numpy.lib.type_check import common_type
import os

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CHECKERBOARD = (8, 11)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * 45
counter = 0
counter2 = 1

imgs = [6,7,21,22,28,29,30,33,46,50,51,52,53,54,55,56,57,58,59,60]

objPoints = []
imgPointsLeft = []
imgPointsRight = []

while True:
    leftName = './images/calibration/left/' + str(counter2) + '.jpg'
    rightName = './images/calibration/right/' + str(counter2) + '.jpg'
    counter2= counter2 + 1
    if not os.path.exists(leftName) or not os.path.exists(rightName):
        break 
    imgLeft = cv2.imread(leftName)
    imgRight = cv2.imread(rightName)
    grayLeft = cv2.cvtColor(imgLeft,cv2.COLOR_BGR2GRAY) 
    grayRight = cv2.cvtColor(imgRight,cv2.COLOR_BGR2GRAY)

    retLeft, cornersLeft = cv2.findChessboardCorners(grayLeft, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    retRight, cornersRight = cv2.findChessboardCorners(grayRight, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if retLeft and retRight:
        counter =counter + 1
        
        objPoints.append(objp)

        corners2Left = cv2.cornerSubPix(grayLeft, cornersLeft, (11,11),(-1,-1), criteria)
        
        imgPointsLeft.append(corners2Left)

        corners2Right = cv2.cornerSubPix(grayRight, cornersRight, (11,11),(-1,-1), criteria)
        
        imgPointsRight.append(corners2Right)

        imgLeft = cv2.drawChessboardCorners(imgLeft, CHECKERBOARD, corners2Left, retLeft)
        imgRight = cv2.drawChessboardCorners(imgRight, CHECKERBOARD, corners2Right, retRight)
        cv2.imshow('imgLeft',imgLeft)
        cv2.imshow('imgRight',imgRight)
        cv2.waitKey(1)

cv2.destroyAllWindows()

print("Good images: " + str(counter) + "/" + str(counter2))

# Load leftMatrices and rightMatrices numpy archive files
leftMatrices = np.load("left_intrinsic.npz")
rightMatrices = np.load("right_intrinsic.npz")

# Retrieve camera matrices and distortion coefficients
newCameraMatrixL = leftMatrices["newCameraMatrix"]
distL = leftMatrices["dist"]

newCameraMatrixR = rightMatrices["newCameraMatrix"]
distR = rightMatrices["dist"]

flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC


criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objPoints, imgPointsLeft, imgPointsRight, newCameraMatrixL, distL, newCameraMatrixR, distR, grayLeft.shape[::-1], criteria_stereo, flags)

print("Stereo Calibration Error",retStereo)

print("Camera translation: ",trans)

print("Camera rotation: ",rot)

rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayLeft.shape[::-1], rot, trans, rectifyScale,(0,0))

stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayLeft.shape[::-1], cv2.CV_16SC2)
stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayLeft.shape[::-1], cv2.CV_16SC2)

cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])

cv_file.release()