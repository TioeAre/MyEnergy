#include <iostream>
#include <opencv2/opencv.hpp>
#include "EnergyDetector.h"

bool IF_DEBUG = true;

int main() {
    cv::VideoCapture cap;
    cap.open("/home/tioeare/project/MyEnergy/media/fans.mp4");
    cv::Mat frame;
    Energy example;
    while(cap.isOpened()){
        cap >> frame;
        if(frame.empty()) continue;
        example.detector(frame);
        imshow("frame", frame);
        cv::waitKey(10);
    }
    return 0;
}
