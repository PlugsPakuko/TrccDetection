from ultralytics import YOLO
from  picamera2 import Picamera2
import picamera2.array
import cv2
import math

# Initialize the PiCamera
camera = PiCamera2()

# Set the resolution of the camera
camera.resolution = (640, 480)

# model
model = YOLO("best.pt")

# object classes
classNames = ['Silo', 'ball_blue', 'ball_purple', 'ball_red']

while True:
    # Capture an image from the PiCamera
    with picamera.array.PiRGBArray(camera) as stream:
        camera.capture(stream, format='bgr')
        img = stream.array

    # Run inference on the captured image
    results = model(img, verbose=False, stream=True)

    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # calculate center coordinates
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # put box in cam
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0] * 100)) / 100
            print("Confidence --->", confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])

            # coordinate
            print("Center: ({}, {})".format(center_x, center_y))

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    # Display the image
    cv2.imshow('Webcam', img)
    
    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera
camera.close()
cv2.destroyAllWindows()
