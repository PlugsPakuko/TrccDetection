from ultralytics import YOLO
import cv2
import keyboard

BallBlue = 1
BallPurple = 2
BallRed = 3

cap = cv2.VideoCapture(0)
model = YOLO("best.pt")
classNames = ['Silo', 'ball_blue', 'ball_purple', 'ball_red']

def get_ball(target_ball):
    nearest_target = (None, None)
    min_d = float('inf')

    success, img = cap.read()
    results = model(img, verbose=False, stream=True)

    for r in results:
        box = r.boxes

        if box and classNames[int(box.cls[0])] == classNames[target_ball]:
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

while True:
    print("Waiting...")
    if keyboard.read_key() == 'p':
        coordinate = get_ball(BallBlue)
    else:
        break

cap.release()
cv2.destroyAllWindows()
