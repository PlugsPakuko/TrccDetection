import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale
from PIL import Image, ImageTk


def GetCenter(x, y, Xcenter, Ycenter, error):
    x_range = range(int(Xcenter) - error, int(Xcenter) + error)
    y_range = range(int(Ycenter) - error, int(Ycenter) + error)

    if x in x_range:
        if y in y_range:
            print("Center")
        elif y > Ycenter:
            print("Forward")
        else:
            print("Backward")
    elif x > Xcenter:
        if y in y_range:
            print("Right")
        elif y > Ycenter:
            print("Forward Right")
        else:
            print("Backward Right")
    else:
        if y in y_range:
            print("Left")
        elif y > Ycenter:
            print("Forward Left")
        else:
            print("Backward Left")


class BallDetectorGUI:
    def __init__(self, master, Xcenter, Ycenter):
        self.master = master
        self.Xcenter = Xcenter
        self.Ycenter = Ycenter
        master.title("Ball Detector")

        
        self.param1_scale = Scale(master, from_=1, to=200, label="Param1",
                                  orient=tk.HORIZONTAL, command=self.update_parameters)
        self.param1_scale.set(100)
        self.param1_scale.pack()

        self.param2_scale = Scale(master, from_=1, to=200, label="Param2",
                                  orient=tk.HORIZONTAL, command=self.update_parameters)
        self.param2_scale.set(35)
        self.param2_scale.pack()

        self.min_radius_scale = Scale(master, from_=1, to=200, label="Min Radius",
                                      orient=tk.HORIZONTAL, command=self.update_parameters)
        self.min_radius_scale.set(10)
        self.min_radius_scale.pack()

        self.max_radius_scale = Scale(master, from_=1, to=200, label="Max Radius",
                                      orient=tk.HORIZONTAL, command=self.update_parameters)
        self.max_radius_scale.set(100)
        self.max_radius_scale.pack()

        self.min_dist_scale = Scale(master, from_=1, to=200, label="Min Distance",
                                      orient=tk.HORIZONTAL, command=self.update_parameters)
        self.min_dist_scale.set(90)
        self.min_dist_scale.pack()

        self.cap = cv2.VideoCapture(0)

        self.canvas = tk.Canvas(master, width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                                height=self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.canvas.pack()

        self.update_parameters()

        self.update()

    def update_parameters(self, event=None):
        self.param1 = self.param1_scale.get()
        self.param2 = self.param2_scale.get()
        self.min_radius = self.min_radius_scale.get()
        self.max_radius = self.max_radius_scale.get()
        self.min_dist = self.min_dist_scale.get()

    def update(self):
        ret, frame = self.cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=self.min_dist,
                                        param1=self.param1, param2=self.param2,
                                        minRadius=self.min_radius, maxRadius=self.max_radius)

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")

                for (x, y, r) in circles:
                    GetCenter(x, y, self.Xcenter, self.Ycenter, 10)
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(image)
            photo = ImageTk.PhotoImage(image=image)

            self.canvas.create_image(0, 0, anchor=tk.NW, image=photo)
            self.canvas.image = photo

        self.master.after(10, self.update)

def main():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    if ret:
        #get center pixel
        height, width, channels = frame.shape
        cap.release()
        
        root = tk.Tk()
        gui = BallDetectorGUI(root, width/2, height/2)
        root.mainloop()
    else:
        print("Failed to capture frame")


if __name__ == "__main__":
    main()
