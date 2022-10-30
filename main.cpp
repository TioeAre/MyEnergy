#include <iostream>
#include "opencv2/opencv.hpp"
#include "EnergyDetector.h"
#include "EnergyPredictor.h"
#include "get_time.h"
#include "angle_solve.h"

bool IF_DEBUG = true;
int fps = 0;
double first_time;

int main() {
    cv::VideoCapture cap;
    cap.open("/home/tioeare/project/MyEnergy/media/red.mp4");
    cv::Mat frame;
    EnergyDetector example;
//    EnergyPre pre;
//    angle_solve ang;
//    first_time = current_time();
//    std::deque<float> before_yaw;
//    std::deque<float> before_pitch;
//    std::deque<float> after_yaw;
//    std::deque<float> after_pitch;

    while (cap.isOpened()) {
        cap >> frame;
        if (frame.empty()) continue;
        example.detector(frame);
        fps = get_time_();
//        std::vector<cv::Point2d> armor = {{example.armor[0].x, example.armor[0].y},
//                                          {example.armor[1].x, example.armor[1].y},
//                                          {example.armor[2].x, example.armor[2].y},
//                                          {example.armor[3].x, example.armor[3].y}};
//        cv::Point3f position;
//        ang.predict_yaw_pitch(armor, position);
//        position.x = position.x / 1000; position.y = position.y / 1000; position.z = position.z / 1000;
//
//        if(example.firstSee){ pre.setFirst(ang.tr_yaw, ang.tr_pitch); example.firstSee = false;}
//
//        //绘图
//        before_yaw.push_back(ang.tr_yaw); before_pitch.push_back(ang.tr_pitch);
//        int y = 500; int x = 1000;
//        cv::Mat yaw(y, x, CV_8UC3, cv::Scalar(255, 255, 255)); cv::Mat pitch(y, x, CV_8UC3, cv::Scalar(255, 255, 255));
//        ///预测
//        if (example.ifChange)
//            pre.setZero();
//        else
//            pre.predict(ang.tr_yaw, ang.tr_pitch, position);
//        //绘图
//        after_yaw.push_back(ang.tr_yaw);
//        after_pitch.push_back(ang.tr_pitch);
//        if (before_yaw.size() > 100) {
//            before_yaw.pop_front(); before_pitch.pop_front();
//            after_yaw.pop_front(); after_pitch.pop_front();
//        }
//        if (!before_pitch.empty())
//            for (int i = 0; i < 99; i++) {
//                cv::circle(yaw, cv::Point(i * 10, y / 2 + before_yaw[i] * 8), 3, cv::Scalar(0, 0, 255), -1, 1);
//                cv::circle(yaw, cv::Point(i * 10, y / 2 + after_yaw[i] * 8), 3, cv::Scalar(0, 255, 0), -1, 1);
//                cv::circle(pitch, cv::Point(i * 10, y / 2 + before_pitch[i] * 8), 3, cv::Scalar(0, 0, 255), -1, 1);
//                cv::circle(pitch, cv::Point(i * 10, y / 2 + after_pitch[i] * 8), 3, cv::Scalar(0, 255, 0), -1, 1);
//            }
//        cv::imshow("yaw", yaw);
//        cv::imshow("pitch", pitch);
        cv::waitKey(20);
    }
    return 0;
}
