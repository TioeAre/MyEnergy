#include <iostream>
#include <opencv2/opencv.hpp>
#include "EnergyDetector.h"
#include "get_time.h"

bool IF_DEBUG = true;
int fps = 0;

int main() {
    cv::VideoCapture cap;
    cap.open("/home/tioeare/project/MyEnergy/media/fans.mp4");
    cv::Mat frame;
    Energy example;
    while (cap.isOpened()) {
        cap >> frame;
        if (frame.empty()) continue;
        example.detector(frame);
        fps = get_time_();
        cv::putText(frame, std::to_string(fps), cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.95,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        imshow("frame", frame);
        cv::waitKey(10);
    }
    return 0;
}
