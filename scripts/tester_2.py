import cv2
import numpy as np
import time
import os
import argparse
import shutil

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
R2 = cv_file.getNode("R1").mat()
T = cv_file.getNode("T").mat()

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (500,500)
fontScale              = 3
fontColor              = (0,255,0)
lineType               = 5

lx_slider = 640
ly_slider = 360

rx_slider = 640
ry_slider = 360

cv2.namedWindow("left")
cv2.namedWindow("right")

def onlX(val):
    global lx_slider
    lx_slider = val

def onlY(val):
    global ly_slider
    ly_slider = val

def onrX(val):
    global rx_slider
    rx_slider = val

def onrY(val):
    global ry_slider
    ry_slider = val

cv2.createTrackbar("x", "left" , 640, 1280, onlX)
cv2.createTrackbar("y", "left" , 360, 720, onlY)
cv2.createTrackbar("x", "right" , 640, 1280, onrX)
cv2.createTrackbar("y", "right" , 360, 720, onrY)

def calculateDepth(lx,ly,rx,ry):
    bowl = np.array([lx,ly]).astype(np.float32)
    bowr = np.array([rx,ry]).astype(np.float32)

    pts1 = cv2.undistortPoints(bowl,K1,D1,None,R1,P1).squeeze()
    pts2 = cv2.undistortPoints(bowr,K2,D2,None,R2,P2).squeeze()
    print(lx,ly,rx,ry,pts1[0],pts1[1],pts2[0],pts2[1])
while(True):
    camera1.grab()
    camera2.grab()

    _,frame1 = camera1.retrieve()
    _,frame2 = camera2.retrieve()
    up_points = (1066,600)


    cv2.circle(frame1,(lx_slider,ly_slider),10,(0,0,255),1)
    cv2.circle(frame1,(lx_slider,ly_slider),1,(0,255,0),1)
    cv2.circle(frame2,(rx_slider,ry_slider),10,(0,0,255),1)
    cv2.circle(frame2,(rx_slider,ry_slider),1,(0,255,0),1)
    frame1 = cv2.resize(frame1, up_points, interpolation= cv2.INTER_LINEAR)
    frame2 = cv2.resize(frame2, up_points, interpolation= cv2.INTER_LINEAR)
    
    depth = calculateDepth(lx_slider,ly_slider,rx_slider,ry_slider)
    
    frame1 = cv2.putText(frame1,str(depth), (900,100),font,1,fontColor,lineType)
    cv2.imshow('left',frame1)
    cv2.imshow('right',frame2)
    cv2.waitKey(1)

camera1.release()
camera2.release()
# Destroy all the windows
cv2.destroyAllWindows()
