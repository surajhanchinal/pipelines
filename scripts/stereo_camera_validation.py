import cv2
import numpy as np
import os

cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_READ)
stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()
cv_file.release()

counter2 = 1

while True:
    leftName = './images/validation/left/' + str(counter2) + '.jpg'
    rightName = './images/validation/right/' + str(counter2) + '.jpg'
    counter2 += 1

    if not (os.path.exists(leftName) and os.path.exists(rightName)):
        break

    left_image = cv2.imread(leftName, 0)
    right_image = cv2.imread(rightName, 0)

    left_rectified = cv2.remap(left_image, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4)
    right_rectified = cv2.remap(right_image, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4)

    window_size = 10
    min_disparity = 16
    num_disparities = 112 - min_disparity
    stereo = cv2.StereoSGBM_create(minDisparity=min_disparity, numDisparities=num_disparities, blockSize=window_size)

    disparity = stereo.compute(left_rectified, right_rectified)

    disparity_normalized = (disparity - min_disparity) / num_disparities

    cv2.imshow('Disparity Map', disparity_normalized)
    cv2.waitKey(3000) 
    cv2.destroyAllWindows()
