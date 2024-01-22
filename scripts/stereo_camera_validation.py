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
T = cv_file.getNode("T").mat()

counter2 = 1
counter = 0

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CHECKERBOARD = (8, 11)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * 45


def getDepth(uptsL,uptsR):
    ptsL = cv2.undistortPoints(uptsL,K1,D1,R=R1,P=P1).squeeze()
    ptsR = cv2.undistortPoints(uptsR,K2,D2,R=R2,P=P2).squeeze()
    baseline = abs(T[0,0]/1000.0)
    depth = P1[0,0]*(baseline)/(abs(ptsL[:,0] - ptsR[:,0]))
    xl = depth*((ptsL[:,0]  - P1[0,2]))/((P1[0,0])) + baseline/2   
    xr = depth*((ptsR[:,0] - P2[0,2]))/((P1[0,0]))   - baseline/2
    yl = depth*(ptsL[:,1]  - P1[1,2] )/(P1[1,1])
    yr = depth*(ptsR[:,1]  - P2[1,2])/(P2[1,1])
    return xl,yl,depth

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
        corners2Leftn = corners2Left.squeeze()
        corners2Rightn = corners2Right.squeeze()
        x,y,z = getDepth(corners2Leftn,corners2Rightn)
        di = 0
        ai = 7
        ci = 80
        bi = 87
        a = np.array([x[ai],y[ai],z[ai]])
        b = np.array([x[bi],y[bi],z[bi]])
        c = np.array([x[ci],y[ci],z[ci]])
        d = np.array([x[di],y[di],z[di]])
        
        ## y diff
        o1 = d - a
        o2 = c - b
        ## x diff
        o5 = b - a
        o6 = c - d

        #print("dist",np.sqrt(np.sum(np.square(b-a))))
        #print("dist",np.sqrt(np.sum(np.square(d-c))))
        #print("dist",np.sqrt(np.sum(np.square(d-a))))
        #print("dist",np.sqrt(np.sum(np.square(b-c))))

        for i in range(corners2Leftn.shape[0]):
            pt1 = cv2.undistortPoints(corners2Leftn[i],K1,D1,None,R1,P1).squeeze()
            pt2 = cv2.undistortPoints(corners2Rightn[i],K2,D2,None,R2,P2).squeeze()
            error = abs(pt1[1]-pt2[1]) + error
        error = error/len(objp[0])
        print(counter2-1,error)
        cv2.imshow('imgLeft',imgLeft)
        cv2.imshow('imgRight',imgRight)
        cv2.waitKey(1)