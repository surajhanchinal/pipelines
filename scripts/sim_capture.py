import cv2
import numpy as np
import time
import os
import argparse
import shutil



parser = argparse.ArgumentParser();
parser.add_argument('--delete', help="Deletes old images",action="store_true")
parser.add_argument("--validation",help="Use this flag to generate images for validation",action="store_true")

# Parse and print the results
args = parser.parse_args()

origPath = os.getcwd();



if args.delete:
    valid = True
    if(args.validation):
        if(os.path.isdir('./images/validation')):
            os.chdir('./images/validation')
        else:
            valid = False
    else:
        if(os.path.isdir('./images/calibration')):
            os.chdir('./images/calibration')
        else:
            valid = False
    print("Deleting all images")
    if(valid):
        shutil.rmtree(os.getcwd())
    print("Delete done")


os.chdir(origPath)

try:
    os.makedirs('./images/validation/left')
except:
    pass
try:
    os.makedirs('./images/validation/right')
except:
    pass
try:
    os.makedirs('./images/calibration/left')
except:
    pass
try:
    os.makedirs('./images/calibration/right')
except:
    pass



camera1 = cv2.VideoCapture(0,cv2.CAP_V4L2)
camera2 = cv2.VideoCapture(2,cv2.CAP_V4L2)

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

state = 'PAUSE'
counter = 1
countTime = 5

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (500,500)
fontScale              = 10
fontColor              = (0,255,0)
lineType               = 5

startPath = './images/calibration'

if args.validation:
    startPath = './images/validation'


while(True):
    camera1.grab()
    camera2.grab()

    _,frame1 = camera1.retrieve()
    _,frame2 = camera2.retrieve()
    #frame1 = cv2.flip(frame1,0)
    #frame2 = cv2.flip(frame2,0)

    if state == 'START':
        curr_time = time.time()
        countTime = 5 - int(curr_time - prev_time)
        if(countTime <= 0):
            cv2.imwrite(startPath+'/left/'+str(counter)+'.jpg',frame1)
            cv2.imwrite(startPath+'/right/'+str(counter)+'.jpg',frame2)
            counter = counter + 1
            prev_time = time.time()
    frame1 = cv2.putText(frame1,str(counter-1), (1000,100),  font,     3,    fontColor,    lineType)
    frame2 = cv2.putText(frame2,str(counter-1), (1000,100),  font,     3,    fontColor,    lineType)
    frame1 = cv2.putText(frame1,str(int(countTime)), bottomLeftCornerOfText,  font,     fontScale,    fontColor,    lineType)
    frame2 = cv2.putText(frame2,str(int(countTime)), bottomLeftCornerOfText,  font,     fontScale,    fontColor,    lineType)
    up_points = (1066,600)
    frame1 = cv2.resize(frame1, up_points, interpolation= cv2.INTER_LINEAR)
    frame2 = cv2.resize(frame2, up_points, interpolation= cv2.INTER_LINEAR)
    cv2.imshow('left',frame1)
    cv2.imshow('right',frame2)
    pressed = cv2.waitKey(1) & 0xFF
    if pressed == ord('s'):
        state = 'START' 
        prev_time = time.time()
    if pressed == ord('p'):
        state = 'PAUSE'
    if pressed == ord('q'):
        break

camera1.release()
camera2.release()
# Destroy all the windows
cv2.destroyAllWindows()
