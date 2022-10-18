#include"angle_solve.h"
#include<cmath>

#define PI 3.1415926535
extern bool IF_DEBUG;
using namespace cv;
using namespace std;

bool judge_armor_type(std::vector<cv::Point2d> armor_mat_points);

angle_solve::angle_solve() {

///8mm内参
//    CameraMatrix = (cv::Mat_<double>(3, 3) << 1788.5, 0, 640.7891,//1788.5, 646.7891,
//            0, 1788.4, 503.5691,//1788.4, 512.5691,
//            0, 0, 1);
//    DistCoeffs = (cv::Mat_<double>(1, 5) << -0.0675, 0.2944, 0, 0, 0.1674);

///6mm内参
    CameraMatrix = (cv::Mat_<double>(3, 3) << 1317.0, 0, 640.1112,//1788.5, 646.7891,
            0, 1317.1, 512.0266,//1788.4, 503.5691,
            0, 0, 1);
    DistCoeffs = (cv::Mat_<double>(1, 5) << -0.0795, 0.1748, 0, 0, 0.0384);
//    Camrea_to_yuntai_T = (cv::Mat_<double>(1, 3) << 0, 38, 98.3);//54
    Camrea_to_yuntai_T = (cv::Mat_<double>(1, 3) << 0, 9, 120.9);
//    Camrea_to_yuntai_T = (cv::Mat_<double>(1, 3) << 0, 40, 90);//54
//    Camrea_to_yuntai_T = (cv::Mat_<double>(1, 3) << 0, 0, 0);
    double b_w = 235, s_w = 135;
    double b_h = 62, s_h = 60;
///大装甲板四点坐标
    armor_world_points_big.push_back(
            cv::Point3d(-b_w / 2.0, b_h / 2.0, 0.0));
    armor_world_points_big.push_back(
            cv::Point3d(-b_w / 2.0, -b_h / 2.0, 0.0));
    armor_world_points_big.push_back(
            cv::Point3d(b_w / 2.0, -b_h / 2.0, 0.0));
    armor_world_points_big.push_back(
            cv::Point3d(b_w / 2.0, b_h / 2.0, 0.0));

///小装甲板四点坐标
    armor_world_points_small.push_back(
            cv::Point3d(-s_w / 2.0, s_h / 2.0, 0.0));
    armor_world_points_small.push_back(
            cv::Point3d(-s_w / 2.0, -s_h / 2.0, 0.0));
    armor_world_points_small.push_back(
            cv::Point3d(s_w / 2.0, -s_h / 2.0, 0.0));
    armor_world_points_small.push_back(
            cv::Point3d(s_w / 2.0, s_h / 2.0, 0.0));
}

///解算云台坐标系下装甲板坐标
Point3f angle_solve::mat_to_yuntai(std::vector<cv::Point2d> armor_mat_points) {

    Point3f camera_points, yuntai_points, last_camera_points;
    vector<Point3d> armor_world_points;

    bool armor_size = judge_armor_type(armor_mat_points);
    if (armor_size == true) {
        solvePnP(armor_world_points_big, armor_mat_points, CameraMatrix, DistCoeffs, R, T);
    } else {
        solvePnP(armor_world_points_small, armor_mat_points, CameraMatrix, DistCoeffs, R, T);
    }

    armor_mat_points.clear();

    camera_points.x = T.at<double>(0);
    camera_points.y = T.at<double>(1);
    camera_points.z = T.at<double>(2);

    yuntai_points.x = camera_points.x + Camrea_to_yuntai_T.at<double>(0, 0);
    yuntai_points.y = camera_points.y + Camrea_to_yuntai_T.at<double>(0, 1);
    yuntai_points.z = camera_points.z + Camrea_to_yuntai_T.at<double>(0, 2);

    return yuntai_points;
}

void angle_solve::predict_yaw_pitch(std::vector<cv::Point2d> armor_mat_points, cv::Point3d &position,
                                    double &enemy_distance) {

    double time = 1;
    double pred = 1;

    Point3f yuntai_point = mat_to_yuntai(armor_mat_points);
    position = yuntai_point;
    double high, length;
    double distance = sqrt((double) (yuntai_point.x * yuntai_point.x) + (yuntai_point.y * yuntai_point.y) +
                           (yuntai_point.z * yuntai_point.z));
    enemy_distance = distance;

    high = yuntai_point.y;
    tr_pitch = asin(high / distance) * (180 / PI);
    tr_yaw = atan2(yuntai_point.x, yuntai_point.z) * (180 / PI);
}

void angle_solve::predict_yaw_pitch(std::vector<cv::Point2d> armor_mat_points, Point3f &position) {
    double distance;
    Point3f yuntai_point = mat_to_yuntai(armor_mat_points);
    position = yuntai_point;
    distance = sqrt((double) (yuntai_point.x * yuntai_point.x) + (yuntai_point.y * yuntai_point.y) +
                    (yuntai_point.z * yuntai_point.z));

    tr_pitch = (float) asin(yuntai_point.y / distance) * (180.0 / PI);
    tr_yaw = (float) atan(yuntai_point.x / yuntai_point.z) * (180.0 / PI);
}


/*
 * @brief judge armor's type
 * @input[armor_mat_points]: img_pts
 * @return[bool]: true->big , false->small
 * @return[armor_mat_points]: correct points for angle_solve()
*/
bool judge_armor_type(std::vector<cv::Point2d> armor_mat_points) {
    int step0[4][2] = {{-1, -1},
                       {-1, 1},
                       {1,  1},
                       {1,  -1}};
    double l_1 = sqrt(pow((armor_mat_points[0].x - armor_mat_points[1].x), 2) +
                      pow((armor_mat_points[0].y - armor_mat_points[1].y), 2));
    double l_2 = sqrt(pow((armor_mat_points[0].x - armor_mat_points[2].x), 2) +
                      pow((armor_mat_points[0].y - armor_mat_points[2].y), 2));
    double l_3 = sqrt(pow((armor_mat_points[0].x - armor_mat_points[3].x), 2) +
                      pow((armor_mat_points[0].y - armor_mat_points[3].y), 2));

    bool big_or_small = false;
    Point2d center_point;
    vector<cv::Point2d> corrected_armor_mat_points;
    corrected_armor_mat_points.resize(4);
    double range = 3.1;

    std::vector<std::pair<int, double>> list{{1, l_1},
                                             {2, l_2},
                                             {3, l_3}};
    std::sort(list.begin(), list.end(),
              [](auto &&left, auto &&right) -> bool { return left.second > right.second; }); //从大到小排序

    center_point.x = (armor_mat_points[0].x + armor_mat_points[list[0].first].x) / 2;
    center_point.y = (armor_mat_points[0].y + armor_mat_points[list[0].first].y) / 2;
    int i = 0;
    for (auto &&iter: corrected_armor_mat_points) {
        iter.x = center_point.x + step0[i][0] * list[1].second / 2;
        iter.y = center_point.y + step0[i][1] * list[2].second / 2;
        i++;
    }
    armor_mat_points = corrected_armor_mat_points;
    if (list[1].second / list[2].second > range) return true;
    else return false;
}
