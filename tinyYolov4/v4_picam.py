from picamera2 import Picamera2
import cv2 as cv
import time

cap = Picamera2()
cap.start()

Conf_threshold = 0.8
NMS_threshold = 0.4

COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]

class_name = ['BlueBall', 'PurpleBall', 'RedBall', 'Silo']

net = cv.dnn.readNet('custom-yolov4-tiny-detector_last.weights', 'custom-yolov4-tiny-detector.cfg')
#net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
#net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
model = cv.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1/255, swapRB = True)

while True:
    frame = cap.capture_array("main")
    rgb_img = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

    classes, scores, boxes = model.detect(rgb_img, Conf_threshold, NMS_threshold)

    for (classid, score, box) in zip(classes, scores, boxes):
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_name[classid], score)
        print(label)
        print(box[0], box[1])
        print("\n")
        cv.rectangle(rgb_img, box, color, 1)
        cv.putText(rgb_img, label, (box[0], box[1]-10),
                   cv.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
    
    cv.imshow('frame', rgb_img)
    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
