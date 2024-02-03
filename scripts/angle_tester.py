import cv2
import numpy as np
import time
import os
import argparse
from scipy.spatial.transform import Rotation   
import shutil
from camera_reader import getCameraPaths

def rot_x(theta):
    theta = theta*(np.pi/180)
    """Returns the 4x4 rotation matrix around the X-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_y(theta):
    theta = theta*(np.pi/180)
    """Returns the 4x4 rotation matrix around the Y-axis."""
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    theta = theta*(np.pi/180)
    """Returns the 4x4 rotation matrix around the Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

leftCam,rightCam = getCameraPaths()

camera1 = cv2.VideoCapture(leftCam,cv2.CAP_V4L2)
camera2 = cv2.VideoCapture(rightCam,cv2.CAP_V4L2)
cv_file = cv2.FileStorage('stereoParams.xml', cv2.FILE_STORAGE_READ)

# Setting Motion Codecs
camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
camera2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
camera2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))

#Seting frame H and W 
camera1.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
camera2.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


K1 = cv_file.getNode("K1").mat()
K2 = cv_file.getNode("K2").mat()
D1 = cv_file.getNode("D1").mat()
D2 = cv_file.getNode("D2").mat()
P1 = cv_file.getNode("P1").mat()
P2 = cv_file.getNode("P2").mat()
R1 = cv_file.getNode("R1").mat()
R2 = cv_file.getNode("R2").mat()
T = cv_file.getNode("T").mat()

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (500,500)
fontScale              = 3
fontColor              = (0,255,0)
lineType               = 5

x_slider = 0
y_slider = 0
z_slider = 0
x_offset = 0
cv2.namedWindow("left")
cv2.namedWindow("right")


def onXOffset(val):
    global x_offset
    x_offset = (val-500)/100

def onX(val):
    global x_slider
    x_slider = (val-500)/50

def onY(val):
    global y_slider
    y_slider = (val-500)/50

def onZ(val):
    global z_slider
    z_slider = (val-500)/50

cv2.createTrackbar("x_offset", "left" , 500, 1000, onXOffset)
cv2.createTrackbar("x", "left" , 500, 1000, onX)
cv2.createTrackbar("y", "left" , 500, 1000, onY)
cv2.createTrackbar("z", "left" , 500, 1000, onZ)


def project3dTo2d(K, D, R, T, isLeft, x, y, z):
    global x_slider,y_slider,z_slider
    newxt = (rot_z(z_slider)@(rot_y(y_slider)@rot_x(x_slider))) @ np.array([x,y,z,1])
    x = newxt[0]
    y = newxt[1]
    z = newxt[2]

    baseline = T[0, 0] / 1000.0  # input is in mm

    sgn = -1 if isLeft else 1

    xx = (x + sgn * (baseline / 2.0)) / z
    yy = y / z

    newOld = np.linalg.inv(R) @ np.array([[xx], [yy], [1]])

    pt = cv2.projectPoints(newOld,np.zeros((3,1)),np.zeros((3,1)),K,D)[0].squeeze()
    return (int)(pt[0]), (int)(pt[1])


while(True):
    camera1.grab()
    camera2.grab()

    _,frame1 = camera1.retrieve()
    _,frame2 = camera2.retrieve()
    up_points = (1066,600)


    #xc = 640
    #xc = int(xc)
    #yc = 360    
    #yc = int(yc)

    #xl,yl = project3dTo2d(K1,D1,R1,T,True,x_slider,y_slider,z_slider)
    #xr,yr = project3dTo2d(K2,D2,R2,T,False,x_slider,y_slider,z_slider)
    
    def draw_line1(frame,x1,y1,z1,x2,y2,z2):
        x1l,y1l = project3dTo2d(K1,D1,R1,T,True,x1,y1,z1)
        x2l,y2l = project3dTo2d(K1,D1,R1,T,True,x2,y2,z2)
        cv2.line(frame,(x1l,y1l),(x2l,y2l),(0,0,255),1)
    
    def draw_line2(frame,x1,y1,z1,x2,y2,z2):
        x1l,y1l = project3dTo2d(K2,D2,R2,T,False,x1,y1,z1)
        x2l,y2l = project3dTo2d(K2,D2,R2,T,False,x2,y2,z2)
        cv2.line(frame,(x1l,y1l),(x2l,y2l),(0,0,255),1)
    

    

    #cv2.circle(frame1,(xl,yl),1,(0,0,255),3)
    #cv2.circle(frame1,(xc,yc),1,(0,255,255),3)
    #cv2.circle(frame2,(xr,yr),1,(0,0,255),3)
    #cv2.circle(frame2,(xc,yc),1,(0,255,255),3)
    
    #draw_line1(frame1,-2,1.37,5.58,2,1.37,5.58)
    #draw_line1(frame1,-2,1.37,6.58,2,1.37,6.58)
    #draw_line1(frame1,-2,1.37,9.83,2,1.37,9.83)

    #draw_line2(frame2,-2,1.37,5.58,2,1.37,5.58)
    #draw_line2(frame2,-2,1.37,6.58,2,1.37,6.58)
    #draw_line2(frame2,-2,1.37,9.83,2,1.37,9.83)

    #draw_line1(frame1,-1,0.73,4.49,1,0.73,4.49)
    #draw_line2(frame2,-1,0.73,4.49,1,0.73,4.49)
    #draw_line1(frame1,-1,1.16,4.49,1,1.16,4.49)
    #draw_line2(frame2,-1,1.16,4.49,1,1.16,4.49)
        

    width = 0.995
    height = 1.035
    distance = 16.8 + x_offset
    groundHeight = 1.73
    fix_x_offset = -0.47
    x1 = 0 + fix_x_offset
    y1 = groundHeight - height
    z1 = distance

    x2 = width + fix_x_offset
    y2 = groundHeight - height
    z2 = distance

    x3 = width + fix_x_offset
    y3 = groundHeight
    z3 = distance

    x4 = 0 + fix_x_offset
    y4 = groundHeight
    z4 = distance
    
    draw_line1(frame1,x1,y1,z1,x2,y2,z2)
    draw_line1(frame1,x2,y2,z2,x3,y3,z3)    
    draw_line1(frame1,x3,y3,z3,x4,y4,z4)    
    draw_line1(frame1,x4,y4,z4,x1,y1,z1)    

    draw_line2(frame2,x1,y1,z1,x2,y2,z2)
    draw_line2(frame2,x2,y2,z2,x3,y3,z3)    
    draw_line2(frame2,x3,y3,z3,x4,y4,z4)    
    draw_line2(frame2,x4,y4,z4,x1,y1,z1)    
    
    new_x_offset = 0.42
    x21 = 0 + fix_x_offset + new_x_offset
    y21 = groundHeight
    z21 = distance
    
    x22 = 0 + fix_x_offset + new_x_offset + 0.2
    y22 = groundHeight
    z22 = distance

    x23 = 0 + fix_x_offset + new_x_offset
    y23 = groundHeight
    z23 = distance + 0.25
    
    x24 = 0 + fix_x_offset + new_x_offset + 0.2
    y24 = groundHeight
    z24 = distance + 0.25

    draw_line1(frame1,x21,y21,z21,x22,y22,z22)
    draw_line1(frame1,x22,y22,z22,x24,y24,z24)    
    draw_line1(frame1,x23,y23,z23,x21,y21,z21)    
    draw_line1(frame1,x24,y24,z24,x23,y23,z23)    

    draw_line2(frame2,x21,y21,z21,x22,y22,z22)
    draw_line2(frame2,x22,y22,z22,x24,y24,z24)    
    draw_line2(frame2,x23,y23,z23,x21,y21,z21)    
    draw_line2(frame2,x24,y24,z24,x23,y23,z23)    


    draw_line1(frame1,-0.3,groundHeight,3,-0.3,groundHeight,10)
    draw_line2(frame2,-0.3,groundHeight,3,-0.3,groundHeight,10)
    #draw_line2(frame2,x4,y4,z4,x4,y4,6.61)
    #draw_line2(frame2,x3,y3,z3,x3,y3,6.54)
    #draw_line1(frame1,x4,y4,z4,x4,y4,6.61)
    #draw_line1(frame1,x3,y3,z3,x3,y3,6.54)

    frame1 = cv2.resize(frame1, up_points, interpolation= cv2.INTER_LINEAR)
    frame2 = cv2.resize(frame2, up_points, interpolation= cv2.INTER_LINEAR)

    frame1 = cv2.putText(frame1,str(x_slider), (900,100),  font,     1,    fontColor,    lineType)
    frame1 = cv2.putText(frame1,str(y_slider), (900,200),  font,     1,    fontColor,    lineType)
    frame1 = cv2.putText(frame1,str(z_slider), (900,300),  font,     1,    fontColor,    lineType)
    cv2.imshow('left',frame1)
    cv2.imshow('right',frame2)
    cv2.waitKey(1)

camera1.release()
camera2.release()
# Destroy all the windows
cv2.destroyAllWindows()
