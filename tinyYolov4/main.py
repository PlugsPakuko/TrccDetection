import cv2
import time
import serial
from picamera2 import Picamera2


#serial comms
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.reset_input_buffer()

#adjust threshold here
Conf_threshold = 0.8
NMS_threshold = 0.4

COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]


net = cv2.dnn.readNet('custom-yolov4-tiny-detector_best.weights', 'custom-yolov4-tiny-detector.cfg')
# net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
# net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)

#Camera Config
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1/255, swapRB = True)

cap = Picamera2()
mode = cap.sensor_modes[4]
config = cap.create_video_configuration(raw=mode)
cap.configure(config)
# print(mode)

CenterX, CenterY = [-1, -1]
BallXRange =[200, 500]
BallYRange = [440, 720]

BlueBall = 0
PurpleBall = 1
RedBall = 2
Silo = 3
class_name = ['BlueBall', 'PurpleBall', 'RedBall', 'Silo']

def get_ball(TargetBall):
    nearest_target = None
    min_d = 0

    frame = cap.capture_array("main")
    rgb_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    classes, scores, boxes = model.detect(rgb_img, Conf_threshold, NMS_threshold)

    for (classid, score, box) in zip(classes, scores, boxes):
        if class_name[int(classid)] == class_name[TargetBall]:
            x1, y1= box[0], box[1]
            x2, y2 = x1 + box[2], y1 + box[3]
            centroid_x = (x1 + x2) / 2
            centroid_y = (y1 + y2) / 2

            if centroid_y > min_d:
                min_d = centroid_y
                nearest_target = (centroid_x, centroid_y)

                # Draw bounding box
                cv2.rectangle(rgb_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    # return rgb_img
    cv2.imshow('frame', rgb_img)
    key = cv2.waitKey(1)
    if(key == ord('q')):
        exit(1)
    if nearest_target is not None:
        return nearest_target
    else:
        return -1


def CountBallSilo(TargetBall):
    Stack = [-1, -1, -1]  # Assuming 3 levels (Bottom, Mid, Top)
    BallData = []  # (Ball, y1)
    silo_box = None

    frame = cap.capture_array("main")
    rgb_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    classes, scores, boxes = model.detect(rgb_img, Conf_threshold, NMS_threshold)

    for (classid, score, box) in zip(classes, scores, boxes):
        if class_name[int(classid)] == "Silo":
            silo_box = box
            silo_x1, silo_y1 = box[0], box[1]
            silo_x2, silo_y2 = silo_x1 + box[2], silo_y1 + box[3]
            silo_height = box[2]

        elif class_name[int(classid)] in class_name:
            BallData.append([int(classid), box[1]])

        cv2.rectangle(rgb_img, box, (0, 255, 0), 2)

    cv2.imshow('frame', rgb_img)
    key = cv2.waitKey(1)
    if(key == ord('q')):
        exit(1)
    if silo_box is not None and len(BallData) >= 1:
        print("FOund Silo\n")
        BallData.sort(key=lambda x: x[1])
        for i in range(min(3, len(BallData))):
            Stack[i] = BallData[i][0]
        return Stack#, rgb_img
    else:
        return -1

def PlaceSiloStrategy(Stack, TargetBall):
    EnemyBall = 1

    if TargetBall == 0:
        EnemyBall = 2
    elif TargetBall == 2:
        EnemyBall = 0

    if(Stack.count(-1) == 1 or Stack.count(-1) == 3):
        return 1
        #place
    elif(Stack.count(TargetBall)+Stack.count(EnemyBall) == 3):
        #move
        return 0
    else:
        #other stra
        pass


def Speed(Coordinate):
    data = "1," + str(Coordinate[0]) +','+ str(Coordinate[1]) + "\n"
    ser.write(bytes(data, 'utf-8'))

def Coor(coordinate):
    data = "2," + str(int(coordinate[0]) - CenterX) + "\n"
    ser.write(bytes(data, 'utf-8'))

def Rotate(Coordinate):
    data = "3," + str(int(Coordinate[0]) - CenterX) + "\n"
    ser.write(bytes(data, 'utf-8'))

def SetOrigin():
    data = "0,0\n"
    ser.write(bytes(data, 'utf-8'))

def Back2Origin():
    data = "2,0,0\n"
    ser.write(bytes(data, 'utf-8'))

def PlaceSilo(Strat):
    data = str(Strat) + "\n"
    ser.write(bytes(data, 'utf-8'))


if __name__ == '__main__':
    cap.start()
    frame = cap.capture_array("main")

    CenterX, CenterY = (frame.shape)[1] /2, (frame.shape)[0]/2
    ser.flushOutput()
    ser.flushInput()
    ser.reset_input_buffer()
    command = [None, None]
    while(1):
        ret = 0
        line = ser.readline().decode('utf-8').rstrip()
        command = line.split(',') #split with whiteSpcae
        print('wating for cmd')
        if(len(command) == 1 and command[0] == '0'):
            SetOrigin()
        elif(len(command) == 2 and command[0] == '1'): #GetBall
            ret = 0
            while(ret == 0):
                Coordinate = get_ball(int(command[1]))
                if(Coordinate != -1):
                    print(Coordinate)
                    if(Coordinate[0] > BallXRange[1] or Coordinate[0] < BallXRange[0]): #rotate
                        Rotate(Coordinate)
                    elif(Coordinate[1] > BallXRange[1] or Coordinate[1] < BallXRange[0]): #speed
                        Speed(Coordinate)
                    else:
                        ser.write(bytes("1,99", 'utf-8'))
                        time.sleep(0.1) #wait for taking
                        Back2Origin()
                        command = [None, None]
                        ret = 1 #ball infront
                        cv2.destroyAllWindows()
                        ser.flushOutput()
                        break

        elif(len(command) == 2 and command[0] == '2'): #CheckSilo
            ret = 0
            while(ret == 0):
                Stack = CountBallSilo(int(command[1]))
                if(Stack != -1):
                    print(Stack)
                    strat = PlaceSiloStrategy(Stack, int(command[1]))
                    PlaceSilo(strat)
                    command = [None, None]
                    cv2.destroyAllWindows()
                    ret = 1
                    cv2.destroyAllWindows()
                    ser.flushOutput()
                    break
        elif(len(command) == 1 and command[0] == '3'):
            pass
        else:
            pass

    cv2.destroyAllWindows()