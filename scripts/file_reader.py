import cv2

vidcap = cv2.VideoCapture('../national_long.mp4')
fps = vidcap.get(cv2.CAP_PROP_FPS)


ret,frame = vidcap.read()

print(frame.size)

print(f"{fps} frames per second")

