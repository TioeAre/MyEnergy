////
//// Created by tioeare on 10/17/22.
////
//
//#ifndef MYENERGY_ENERGYPREDICTOR_H
//#define MYENERGY_ENERGYPREDICTOR_H
//
//#include "iostream"
//#include "deque"
//#include "math.h"
//#include <opencv2/core.hpp>
//#include "Eigen/Core"
//#include <ceres/ceres.h>
//
//extern int fps;
//
//class EnergyPre {
//public:
//    /**
//     * @brief 预测下一刻打击时的yaw和pitch
//     * @param yaw 当前的yaw，从电控获得
//     * @param pitch 当前的pitch
//     * @param armor_pos 目标装甲板中心点的云台坐标系坐标(需要在函数内转化为底盘坐标系坐标)
//     */
//    void predict(float &yaw, float &pitch, cv::Point3f armor_pos);
//
//    /**
//     * @brief 切换装甲板时清零之前的存储
//     */
//    void setZero() {armorPoints.resize(0); times.resize(0); speeds.resize(0);}
//    void setFirst(float yaw, float pitch){firstYaw = yaw; firstPitch = pitch;}
//    EnergyPre(){};
//private:
//    int size = 35;
//    int gap = 1;
//    float firstYaw, firstPitch;   //第一次识别到能量机关时云台的yaw和pitch
//    float yawNow = 0;
//    float pitchNow = 0; //当前的云台相对于第一次识别到能量机关时的yaw和pitch值
//    float prediction = 1 / (float) fps; //TODO:调整
//    Eigen::Vector3f normVec;    //能量机关旋转面的法向量
//    cv::Point3f centerPoint;   //能量机关中心点的三维坐标
//    cv::Point3f armorWorldPoint;   //装甲板世界坐标系的三维坐标
//    std::deque<cv::Point3f> armorPoints;    //存放过去100个装甲板中心点世界坐标系三维坐标的容器
//    std::deque<float> times;    //与速度相对应的时间
//    std::deque<float> speeds;   //每次观测到的速度
//    double abc[3] = {0.785, 1.884, 1.305};
//    //损失函数
//    struct CURVE_FITTING_COST {
//        CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
//        template<typename T>
//        bool operator()(const T *const abc, T *residual) const {
//            residual[0] = T(_y) - (abc[0] * ceres::sin(abc[1] * T(_x)) + abc[2]); // y-a * sin(bx) + c
//            return true;
//        }
//        const double _x, _y;
//    };
//    /**
//     * @brief 转换云台坐标系到底盘的世界坐标系或世界坐标系到云台坐标系
//     * @param armor 云台坐标系下装甲板的坐标
//     * @return
//     */
//    cv::Point3f toWorldPoints(cv::Point3f armor);
//    cv::Point3f toWorldPoints(Eigen::Vector3f armor);
//    /**
//     * @brief 拟合三维圆轨迹
//     * @return 能量机关圆心坐标
//     */
//    cv::Point3f fixCenter();
//    /**
//     * @brief 计算速度
//     * @param i deque中哪一时刻
//     * @return
//     */
//    float calSpeed(int i);
//    /**
//     * @brief 计算两点角度
//     * @return 角度
//     */
//    float calAngle(Eigen::Vector3f point1, Eigen::Vector3f point2);
//    /**
//     * @brief 最小二乘求参数
//     */
//    void solve();
//};
//
//#endif //MYENERGY_ENERGYPREDICTOR_H
