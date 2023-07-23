import cv2
import numpy as np
import os

cv_file = cv2.FileStorage('stereoParams.xml',cv2.FILE_STORAGE_READ)

K1 = cv_file.getNode("K1").mat()
D1 = cv_file.getNode("D1").mat()
K2 = cv_file.getNode("K2").mat()
D2 = cv_file.getNode("D2").mat()
R1 = cv_file.getNode("R1").mat()
R2 = cv_file.getNode("R2").mat()
P1 = cv_file.getNode("P1").mat()
P2 = cv_file.getNode("P2").mat()
F = cv_file.getNode("F").mat()

counter2 = 1
counter = 0

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CHECKERBOARD = (8, 11)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * 45


while True:
    leftName = './images/validation/left/' + str(counter2) + '.jpg'
    rightName = './images/validation/right/' + str(counter2) + '.jpg'
    counter2 = counter2 + 1
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
        
        #objPoints.append(objp)

        corners2Left = cv2.cornerSubPix(grayLeft, cornersLeft, (11,11),(-1,-1), criteria)
        
        #imgPointsLeft.append(corners2Left)

        corners2Right = cv2.cornerSubPix(grayRight, cornersRight, (11,11),(-1,-1), criteria)
        
        #imgPointsRight.append(corners2Right)

        imgLeft = cv2.drawChessboardCorners(imgLeft, CHECKERBOARD, corners2Left, retLeft)
        imgRight = cv2.drawChessboardCorners(imgRight, CHECKERBOARD, corners2Right, retRight)
        ilen = len(objp)
        error = 0
        for i in range(len(objp[0])):
            pt1 = cv2.undistortPoints(corners2Left[i][0],K1,D1,None,R1,P1)
            pt2 = cv2.undistortPoints(corners2Right[i][0],K2,D2,None,R2,P2)
            error = abs(pt1[0][0][1]-pt2[0][0][1]) + error
        error = error/len(objp[0])
        print(counter2-1,error)
        cv2.imshow('imgLeft',imgLeft)
        cv2.imshow('imgRight',imgRight)
        cv2.waitKey(3000)