import cv2
import numpy as np
from numpy.lib.type_check import common_type
import os
import math

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CHECKERBOARD = (8, 11)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * 45
counter = 0
counter2 = 1

def rotationMatrixToEulerAngles(R) : 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x*(180/math.pi), y*(180/math.pi), z*(180/math.pi)])

scale_factor = 0.7



objPoints = []
imgPointsLeft = []
imgPointsRight = []

ignore_list = []

while True:
    leftName = './images/calibration/left/' + str(counter2) + '.jpg'
    rightName = './images/calibration/right/' + str(counter2) + '.jpg'
    skip = False
    for x in ignore_list:
        if x == counter2:
            skip = True
            break
    if skip:
        counter2 = counter2 + 1
        continue
    if not os.path.exists(leftName) or not os.path.exists(rightName):
        break 
    imgLeft = cv2.imread(leftName)
    imgRight = cv2.imread(rightName)
    grayLeft = cv2.cvtColor(imgLeft,cv2.COLOR_BGR2GRAY) 
    grayRight = cv2.cvtColor(imgRight,cv2.COLOR_BGR2GRAY)

    retLeft, cornersLeft = cv2.findChessboardCorners(grayLeft, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    retRight, cornersRight = cv2.findChessboardCorners(grayRight, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if retLeft and retRight:
        corners2Left = cv2.cornerSubPix(grayLeft, cornersLeft, (11,11),(-1,-1), criteria)
        corners2Right = cv2.cornerSubPix(grayRight, cornersRight, (11,11),(-1,-1), criteria)
        up_points = (1066,600)

        imgLeft = cv2.drawChessboardCorners(imgLeft, CHECKERBOARD, corners2Left, retLeft)
        imgLeftSmol = cv2.resize(imgLeft,up_points,interpolation= cv2.INTER_LINEAR)
        imgRight = cv2.drawChessboardCorners(imgRight, CHECKERBOARD, corners2Right, retRight)
        imgRightSmol = cv2.resize(imgRight,up_points,interpolation= cv2.INTER_LINEAR)
        cv2.imshow('imgLeft',imgLeftSmol)
        cv2.imshow('imgRight',imgRightSmol)
        pressed = cv2.waitKey(0) & 0xFF
        #if(True):
        if(pressed == ord('y')):
            counter =counter + 1
            objPoints.append(objp)
            imgPointsLeft.append(corners2Left)
            imgPointsRight.append(corners2Right)
        else:
            print("bad image: ",counter2)
    counter2= counter2 + 1
cv2.destroyAllWindows()

print("Good images: " + str(counter) + "/" + str(counter2))

left_camera_params = cv2.FileStorage('left_params.xml', cv2.FILE_STORAGE_READ)
right_camera_params = cv2.FileStorage('right_params.xml', cv2.FILE_STORAGE_READ)


# Retrieve camera matrices and distortion coefficients
onewCameraMatrixL = left_camera_params.getNode("cameraMatrix").mat()
odistL = left_camera_params.getNode("distCoeffs").mat()

onewCameraMatrixR = right_camera_params.getNode("cameraMatrix").mat()
odistR = right_camera_params.getNode("distCoeffs").mat()

flags = 0
flags |= cv2.CALIB_USE_INTRINSIC_GUESS


criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 0.001)

initR = np.identity(3).astype(np.float64)
initT = np.array([[820],[0],[0]],dtype=np.float64)


#retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objPoints, imgPointsLeft, imgPointsRight, newCameraMatrixL, distL, newCameraMatrixR, distR, grayLeft.shape[::-1], criteria_stereo, flags)
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix,rvecs,tvecs,perViewErrors = cv2.stereoCalibrateExtended(objPoints, imgPointsLeft, imgPointsRight,onewCameraMatrixL,odistL,onewCameraMatrixR,odistR,grayLeft.shape[::-1],initR,initT,flags=flags)

print(retStereo)
print(rot)
print(trans)

rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayLeft.shape[::-1], rot, trans)

cv_file = cv2.FileStorage('stereoParams.xml', cv2.FILE_STORAGE_WRITE)

cv_file.write('K1',newCameraMatrixL)
cv_file.write('D1',distL)
cv_file.write('K2',newCameraMatrixR)
cv_file.write('D2',distR)
cv_file.write('E',essentialMatrix)
cv_file.write('F',fundamentalMatrix)
cv_file.write('R',rot)
cv_file.write('T',trans)
cv_file.write('P1',projMatrixL)
cv_file.write('R1',rectL)
cv_file.write('P2',projMatrixR)
cv_file.write('R2',rectR)
cv_file.release()
