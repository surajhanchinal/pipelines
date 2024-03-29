import cv2
import numpy as np
import time
import os
import argparse
from scipy.spatial.transform import Rotation   
import shutil

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

camera1 = cv2.VideoCapture(4,cv2.CAP_V4L2)
camera2 = cv2.VideoCapture(2,cv2.CAP_V4L2)
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
y_slider = 1.37
z_slider = 5
cv2.namedWindow("left")
cv2.namedWindow("right")

def onX(val):
    global x_slider
    x_slider = (val-500)/100

def onY(val):
    global y_slider
    y_slider = (val-50)/100

def onZ(val):
    global z_slider
    z_slider = val/100

cv2.createTrackbar("x", "left" , 500, 1000, onX)
cv2.createTrackbar("y", "left" , 187, 350, onY)
cv2.createTrackbar("z", "left" , 500, 4500, onZ)


def project3dTo2d(K, D, R, T, isLeft, x, y, z):

    newxt = (rot_z(0)@(rot_y(0)@rot_x(-2))) @ np.array([x,y,z,1])
    x = newxt[0]
    y = newxt[1]
    z = newxt[2]
    print(x,y,z,newxt)

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


    xc = 640
    xc = int(xc)
    yc = 360    
    yc = int(yc)

    xl,yl = project3dTo2d(K1,D1,R1,T,True,x_slider,y_slider,z_slider)
    xr,yr = project3dTo2d(K2,D2,R2,T,False,x_slider,y_slider,z_slider)
    
    def draw_line1(frame,x1,y1,z1,x2,y2,z2):
        x1l,y1l = project3dTo2d(K1,D1,R1,T,True,x1,y1,z1)
        x2l,y2l = project3dTo2d(K1,D1,R1,T,True,x2,y2,z2)
        cv2.line(frame,(x1l,y1l),(x2l,y2l),(0,255,0),3)
    
    def draw_line2(frame,x1,y1,z1,x2,y2,z2):
        x1l,y1l = project3dTo2d(K2,D2,R2,T,False,x1,y1,z1)
        x2l,y2l = project3dTo2d(K2,D2,R2,T,False,x2,y2,z2)
        cv2.line(frame,(x1l,y1l),(x2l,y2l),(0,255,0),3)
    

    

    cv2.circle(frame1,(xl,yl),1,(0,0,255),3)
    cv2.circle(frame1,(xc,yc),1,(0,255,255),3)
    cv2.circle(frame2,(xr,yr),1,(0,0,255),3)
    cv2.circle(frame2,(xc,yc),1,(0,255,255),3)
    
    draw_line1(frame1,0,1.37,5.58,0,1.37,6.58)

    draw_line2(frame2,0,1.37,5.58,0,1.37,6.58)
    
    draw_line1(frame1,0,0,5.58,0,0,6.58)

    draw_line2(frame2,0,0,5.58,0,0,6.58)
    
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
