#ifndef _ANGLE_SLOVE_
#define _ANGLE_SLOVE_

#include<opencv2/opencv.hpp>
#include "Eigen/Core"

class angle_solve {
public:
    int wrong_pred = 0;
    double former_yaw_pred = 0;
    double former_yaw = 0;
    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;

    cv::Mat R;
    cv::Mat T;
    cv::Mat Camrea_to_yuntai_T;

    std::vector<cv::Point3d> armor_world_points_small;
    std::vector<cv::Point3d> armor_world_points_big;

    float tr_yaw;
    float tr_pitch;

    Eigen::Matrix3d X;

    bool state = false;

    angle_solve();
    /**
    * @brief ?????? \n(????)
    */
//    double predict_yaw_pitch(cv::Point3d center_point, int armor_size, float& yaw, float& pitch_send);
    void predict_yaw_pitch(std::vector<cv::Point2d> armor_mat_points, cv::Point3d &position, double &enemy_distance);
    void predict_yaw_pitch(std::vector<cv::Point2d> armor_mat_points, cv::Point3f &position);
    cv::Point3f mat_to_yuntai(std::vector<cv::Point2d> armor_mat_points);
    std::vector<cv::Point2d> ArmorMatPoints;

    bool info = false;
    cv::Mat clf;
};

#endif
