import cv2

def getCameraPaths():
    camera_assignment_params = cv2.FileStorage('camera_assignment.xml', cv2.FILE_STORAGE_READ)
    leftPath = int(camera_assignment_params.getNode("left_camera_path").real)
    rightPath = int(camera_assignment_params.getNode("right_camera_path").real)
    return leftPath,rightPath