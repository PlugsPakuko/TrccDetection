import cv2
import time
import keyboard

#adjust threshold here
Conf_threshold = 0.8
NMS_threshold = 0.4

COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]


net = cv2.dnn.readNet('custom-yolov4-tiny-detector_last.weights', 'custom-yolov4-tiny-detector.cfg')
# net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
# net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)

model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1/255, swapRB = True)
cap = cv2.VideoCapture(0)

BallBlue = 0
BallPurple = 1
BallRed = 2
Silo = 3

class_name = ['RedBall', 'PurpleBall', 'BlueBall', 'Silo']

def get_ball(TargetBall):
    nearest_target = (None, None)
    min_d = float('inf')

    success, frame = cap.read()
    classes, scores, boxes = model.detect(frame, Conf_threshold, NMS_threshold)

    for (classid, score, box) in zip(classes, scores, boxes):
        if class_name[int(classid)] == class_name[TargetBall]:
            x1, y1= box[0], box[1]
            x2, y2 = x1 + box[2], y1 + box[3]
            centroid_x = (x1 + x2) / 2
            centroid_y = (y1 + y2) / 2

            d = x2 - x1
            if d < min_d:
                min_d = d
                nearest_target = (centroid_x, centroid_y)
                
                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    if nearest_target is not None:
        print("Nearest x:{} y:{}".format(nearest_target[0], nearest_target[1]))
    else:
        print("Ball not found")

    return nearest_target, frame

def CountBallSilo(TargetBall):
    Stack = [-1, -1, -1]  # Assuming 3 levels (Bottom, Mid, Top)
    BallData = []  # (Ball, y1)
    silo_box = None
    success, frame = cap.read()
    classes, scores, boxes = model.detect(frame, Conf_threshold, NMS_threshold)

    for (classid, score, box) in zip(classes, scores, boxes):
        if class_name[int(classid)] == "Silo":
            silo_box = box
            silo_x1, silo_y1 = box[0], box[1]
            silo_x2, silo_y2 = silo_x1 + box[2], silo_y1 + box[3]
            silo_height = box[2]

        elif class_name[int(classid)] in class_name:
            BallData.append([int(classid), box[1]])

        cv2.rectangle(frame, box, (0, 255, 0), 2)

    if BallData: 
        BallData.sort(key=lambda x: x[1])
        for i in range(min(3, len(BallData))):
            Stack[i] = BallData[i][0]
        print("Stack Counts:", Stack)
        return Stack, frame
    else:
        print("Silo not found")
        return -1

while True:
    print('waiting')
    # coordinate, img_with_bbox = get_ball(BallBlue)
    Stack, img_with_bbox = CountBallSilo(BallBlue)
    cv2.imshow("CountBallSilo", img_with_bbox)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()