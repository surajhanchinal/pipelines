import cv2
import glob
import numpy as np

ignore_list = []


def calibrate_camera(images_folder, camera_name,ignore_list=[]):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 10, 1e-5)
    CHECKERBOARD = (8, 11)
    images = glob.glob(images_folder + "/*.jpg")
    counter = 0
    counter2 = 0
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * 45

    objPoints = []
    imgPoints = []

    for path in images:
        for ign in ignore_list:
            if(ign == path):
                continue
        counter2 = counter2 + 1
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow("img", img)
            pressed = cv2.waitKey(0) & 0xFF
            if(pressed == ord('y')):
                counter = counter + 1
                objPoints.append(objp)
                imgPoints.append(corners2)
            else:
                ignore_list.append(path)
              print("bad image: ",path)
        else:
            print("Chessboard corners not found in image: ", path)

    print("Good images: " + str(counter) + "/" + str(counter2))

    # Perform camera calibration
    if counter > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objPoints, imgPoints, gray.shape[::-1], None, None
        )

        print("Reprojection error: ", ret)
        print("Camera matrix:")
        print(mtx)
        print("Distortion coefficients:")
        print(dist)

        # Get optimal new camera matrix for undistortion
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, gray.shape[::-1], 1, gray.shape[::-1]
        )

        print("New Camera Matrix: ")
        print(newCameraMatrix)
        print("ROI: ", roi)

        # Save the calibration parameters to a file
        cv_file = cv2.FileStorage(camera_name+'_params.xml', cv2.FILE_STORAGE_WRITE)
        cv_file.write('cameraMatrix',mtx)
        cv_file.write('distCoeffs',dist)
        cv_file.release()
    else:
        print("Camera calibration failed. Not enough good images.")

if __name__ == "__main__":
    # Calibrate the left camera
    left_images_folder = "./images/calibration/left"
    calibrate_camera(left_images_folder, "left")

    # Calibrate the right camera
    right_images_folder = "./images/calibration/right"
    calibrate_camera(right_images_folder, "right")
