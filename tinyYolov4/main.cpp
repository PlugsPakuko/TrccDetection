#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>

//serial comms
serial::Serial ser("/dev/ttyUSB0", 115200, serial::Timeout::work_around(1));

//adjust threshold here
float Conf_threshold = 0.8;
float NMS_threshold = 0.4;

std::vector<cv::Scalar> COLORS = {{0, 255, 0}, {0, 0, 255}, {255, 0, 0},
                                 {255, 255, 0}, {255, 0, 255}, {0, 255, 255}};

cv::dnn::Net net = cv::dnn::readNet("custom-yolov4-tiny-detector_best.weights", "custom-yolov4-tiny-detector.cfg");
// net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
// net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);

//Camera Config
cv::dnn::DetectionModel model(net);
model.setInputParams(size_t(416), size_t(416), 1.0/255.0, true);

cv::VideoCapture cap(0);
cv::Size mode = cap.get(cv::CAP_PROP_FRAME_SIZE);
// print(mode);

std::vector<int> CenterX = {-1, -1};
std::vector<int> BallXRange = {200, 500};
std::vector<int> BallYRange = {440, 720};

int BlueBall = 0;
int PurpleBall = 1;
int RedBall = 2;
int Silo = 3;
std::vector<std::string> class_name = {"BlueBall", "PurpleBall", "RedBall", "Silo"};

std::pair<float, float> get_ball(int TargetBall) {
    std::pair<float, float> nearest_target = {-1, -1};
    float min_d = 0;

    cv::Mat frame;
    cap >> frame;
    cv::Mat rgb_img;
    cv::cvtColor(frame, rgb_img, cv::COLOR_BGR2RGB);
    std::vector<int> classes;
    std::vector<float> scores;
    std::vector<cv::Rect> boxes;
    model.detect(rgb_img, classes, scores, boxes, Conf_threshold, NMS_threshold);

    for (size_t i = 0; i < classes.size(); ++i) {
        if (class_name[classes[i]] == class_name[TargetBall]) {
            float x1 = boxes[i].x, y1 = boxes[i].y;
            float x2 = x1 + boxes[i].width, y2 = y1 + boxes[i].height;
            float centroid_x = (x1 + x2) / 2;
            float centroid_y = (y1 + y2) / 2;

            if (centroid_y > min_d) {
                min_d = centroid_y;
                nearest_target = {centroid_x, centroid_y};

                // Draw bounding box
                cv::rectangle(rgb_img, cv::Point(x1, y1), cv::Point(x2, y2), COLORS[0], 2);
            }
        }
    }
    cv::imshow("frame", rgb_img);
    int key = cv::waitKey(1);
    if (key == 'q') {
        exit(1);
    }
    return nearest_target;
}

std::vector<int> CountBallSilo(int TargetBall) {
    std::vector<int> Stack = {-1, -1, -1};  // Assuming 3 levels (Bottom, Mid, Top)
    std::vector<std::pair<int, float>> BallData;  // (Ball, y1)
    cv::Rect silo_box;

    cv::Mat frame;
    cap >> frame;
    cv::Mat rgb_img;
    cv::cvtColor(frame, rgb_img, cv::COLOR_BGR2RGB);

    std::vector<int> classes;
    std::vector<float> scores;
    std::vector<cv::Rect> boxes;
    model.detect(rgb_img, classes, scores, boxes, Conf_threshold, NMS_threshold);

    for (size_t i = 0; i < classes.size(); ++i) {
        if (class_name[classes[i]] == "Silo") {
            silo_box = boxes[i];
            float silo_x1 = boxes[i].x, silo_y1 = boxes[i].y;
            float silo_x2 = silo_x1 + boxes[i].width, silo_y2 = silo_y1 + boxes[i].height;
            float silo_height = boxes[i].height;
        } else if (std::find(class_name.begin(), class_name.end(), class_name[classes[i]]) != class_name.end()) {
            BallData.emplace_back(classes[i], boxes[i].y);
        }

        cv::rectangle(rgb_img, boxes[i], COLORS[0], 2);
    }

    cv::imshow("frame", rgb_img);
    int key = cv::waitKey(1);
    if (key == 'q') {
        exit(1);
    }
    if (!silo_box.empty() && !BallData.empty()) {
        std::cout << "Found Silo\n";
        std::sort(BallData.begin(), BallData.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
        for (int i = 0; i < std::min(3, (int)BallData.size()); ++i) {
            Stack[i] = BallData[i].first;
        }
        return Stack;
    } else {
        return {-1, -1, -1};
    }
}

int PlaceSiloStrategy(std::vector<int> Stack, int TargetBall) {
    int EnemyBall = 1;

    if (TargetBall == 0) {
        EnemyBall = 2;
    } else if (TargetBall == 2) {
        EnemyBall = 0;
    }

    if (std::count(Stack.begin(), Stack.end(), -1) == 1 || std::count(Stack.begin(), Stack.end(), -1) == 3) {
        return 1;
        //place
    } else if (std::count(Stack.begin(), Stack.end(), TargetBall) + std::count(Stack.begin(), Stack.end(), EnemyBall) == 3) {
        //move
        return 0;
    } else {
        //other stra
        return -1;
    }
}

void Speed(std::pair<float, float> Coordinate) {
    std::string data = "1," + std::to_string(Coordinate.first) + "," + std::to_string(Coordinate.second) + "\n";
    ser.write(data);
}

void Coor(std::pair<float, float> coordinate) {
    std::string data = "2," + std::to_string(static_cast<int>(coordinate.first) - CenterX[0]) + "\n";
    ser.write(data);
}

void Rotate(std::pair<float, float> Coordinate) {
    std::string data = "3," + std::to_string(static_cast<int>(Coordinate.first) - CenterX[0]) + "\n";
    ser.write(data);
}

void SetOrigin() {
    std::string data = "0,0\n";
    ser.write(data);
}


// Assuming you have the necessary libraries and classes for serial communication, camera capture, and image processing

void PlaceSilo(std::string Strat) {
    std::string data = Strat + "\n";
    ser.write(data.c_str(), data.length());
}

int main() {
    cap.start();
    cv::Mat frame = cap.capture_array("main");

    double CenterX = frame.cols / 2.0;
    double CenterY = frame.rows / 2.0;
    ser.flushOutput();
    ser.flushInput();
    ser.reset_input_buffer();
    std::vector<std::string> command = {nullptr, nullptr};

    while (true) {
        int ret = 0;
        std::string line = ser.readline().trim();
        std::vector<std::string> command = split(line, ',');
        std::cout << "Waiting for cmd" << std::endl;
        if (command.size() == 1 && command[0] == "0") {
            SetOrigin();
        } else if (command.size() == 2 && command[0] == "1") { // GetBall
            ret = 0;
            while (ret == 0) {
                std::vector<int> Coordinate = get_ball(std::stoi(command[1]));
                if (Coordinate != std::vector<int>{-1}) {
                    std::cout << Coordinate << std::endl;
                    if (Coordinate[0] > BallXRange[1] || Coordinate[0] < BallXRange[0]) { // rotate
                        Rotate(Coordinate);
                    } else if (Coordinate[1] > BallXRange[1] || Coordinate[1] < BallXRange[0]) { // speed
                        Speed(Coordinate);
                    } else {
                        ser.write("1,99", 4);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for taking
                        Back2Origin();
                        command = {nullptr, nullptr};
                        ret = 1; // ball in front
                        cv::destroyAllWindows();
                        ser.flushOutput();
                        break;
                    }
                }
            }
        } else if (command.size() == 2 && command[0] == "2") { // CheckSilo
            ret = 0;
            while (ret == 0) {
                int Stack = CountBallSilo(std::stoi(command[1]));
                if (Stack != -1) {
                    std::cout << Stack << std::endl;
                    std::string strat = PlaceSiloStrategy(Stack, std::stoi(command[1]));
                    PlaceSilo(strat);
                    command = {nullptr, nullptr};
                    cv::destroyAllWindows();
                    ret = 1;
                    cv::destroyAllWindows();
                    ser.flushOutput();
                    break;
                }
            }
        } else if (command.size() == 1 && command[0] == "3") {
            // Do nothing
        } else {
            // Do nothing
        }
    }

    cv::destroyAllWindows();
    return 0;
}

