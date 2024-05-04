from ultralytics import YOLO
import cv2
import keyboard

Silo = 0
BallBlue = 1
BallPurple = 2
BallRed = 3

cap = cv2.VideoCapture(0)
model = YOLO("best.pt")
classNames = ['Silo', 'ball_blue', 'ball_purple', 'ball_red']

def get_ball(TargetBall):
    nearest_target = (None, None)
    min_d = float('inf')

    success, img = cap.read()
    results = model(img, verbose=False, stream=True)

    for r in results:
        box = r.boxes

        if box and classNames[int(box.cls[0])] == classNames[TargetBall]:
            x1, y1, x2, y2 = box.xyxy[0]
            centroid_x = ( float(x1) + float(x2) ) / 2
            centroid_y = ( float(y1) + float(y2) ) / 2

            d = float(x2) - float(x1)
            if d < min_d:
                min_d = d
                nearest_target = (centroid_x, centroid_y)
    
    if nearest_target is not None:
        print("Nearest x:{} y:{}".format(nearest_target[0], nearest_target[1]))
    else:
        print("Ball not found")

    return nearest_target

def CountBallSilo(TargetBall):
    # (Silo Existed, Top, Mid, Bottom)
    Stack = (0 ,-1, -1, -1)

    success, img = cap.read()
    results = model(img, verbose=False, stream=True)

    for r in results:
        box = r.boxes

        if box and classNames[int(box.cls[0])] == classNames[Silo] :
            Stack[0] = 1
        elif box and classNames[int(box.cls[0])] == classNames[TargetBall]:
            x1, y1, x2, y2 = box.xyxy[0]
            Stack[(y2 // 160) + 1] = 1
        else:
            x1, y1, x2, y2 = box.xyxy[0]
            Stack[(y2 // 160) + 1] = 0
            
    
    if Stack[0]:
        print("Silo Detected!")
        #strategy
    else:
        print("Silo not found") 
        return -1


while True:
    print("Waiting...")
    if keyboard.read_key() == 'p': 
        coordinate = get_ball(BallBlue)
    else:
        break

cap.release()
cv2.destroyAllWindows()
