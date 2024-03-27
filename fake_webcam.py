import cv2
import pyfakewebcam
import numpy as np

# OpenCV capture device
cap = cv2.VideoCapture(0)

# Define the resolution
width = 1280
height = 720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# Virtual webcam device (ensure the device path matches what v4l2loopback creates)
fake_cam = pyfakewebcam.FakeWebcam('/dev/video2', width, height)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Process frame with OpenCV here (e.g., applying filters)

    # OpenCV uses BGR by default, but pyfakewebcam expects RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Output to virtual webcam
    fake_cam.schedule_frame(frame_rgb)

# Release the capture device
cap.release()