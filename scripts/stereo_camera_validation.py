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
        i1 = np.array([0,7*0.045,0])

        o2 = c - b
        i2 = np.array([0,7*0.045,0])

        ## x diff

        o5 = b - a
        i5 = np.array([10*0.045,0,0])

        o6 = c - d
        i6 = np.array([10*0.045,0,0])

        print("dist",np.sqrt(np.sum(np.square(b-a))))
        print("dist",np.sqrt(np.sum(np.square(d-c))))
        print("dist",np.sqrt(np.sum(np.square(d-a))))
        print("dist",np.sqrt(np.sum(np.square(b-c))))
        print(o1)
        print(o2)
        print(o5)
        print(o6)

        sty1 = -o5[2]/i5[0]
        sty2 = -o6[2]/i6[0] 

        stx11 = o1[2]/(i1[1]*np.cos(np.arcsin(sty1)))
        stx12 = o1[2]/(i1[1]*np.cos(np.arcsin(sty2)))

        stx21 = o2[2]/(i2[1]*np.cos(np.arcsin(sty1)))
        stx22 = o2[2]/(i2[1]*np.cos(np.arcsin(sty2)))

        ttz1 = o5[1]/o5[0]
        ttz2 = o6[1]/o6[0]

        ty1 = np.arcsin(sty1)*(180/np.pi)
        ty2 = np.arcsin(sty2)*(180/np.pi)

        tx11 = np.arcsin(stx11)*(180/np.pi)
        tx12 = np.arcsin(stx12)*(180/np.pi)

        tx21 = np.arcsin(stx21)*(180/np.pi)
        tx22 = np.arcsin(stx22)*(180/np.pi)

        tz1 = np.arctan(ttz1)*(180/np.pi)
        tz2 = np.arctan(ttz2)*(180/np.pi)
        print(o1)
        print(o2)
        print(o5)
        print(o6)
        print(tx11,ty1,tz1)
        print(tx12,ty2,tz1)
        print(tx21,ty1,tz2)
        print(tx22,ty2,tz2)

        maxe = 0

        for i in range(corners2Leftn.shape[0]):
            pt1 = cv2.undistortPoints(corners2Leftn[i],K1,D1,None,R1,P1).squeeze()
            pt2 = cv2.undistortPoints(corners2Rightn[i],K2,D2,None,R2,P2).squeeze()
            print(corners2Leftn[i],corners2Rightn[i],pt1,pt2)
            #print(x[i],y[i],z[i])  
            maxe = max(maxe,pt1[1]-pt2[1])  
            error = abs(pt1[1]-pt2[1]) + error
        error = error/len(objp[0])
        print(maxe)
        print(counter2-1,error)
        cv2.imshow('imgLeft',imgLeft)
        cv2.imshow('imgRight',imgRight)
        cv2.waitKey(30000)
        break