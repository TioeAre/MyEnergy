//
// Created by tioeare on 10/14/22.
//

#ifndef MYENERGY_ENERGYDETECTOR_H
#define MYENERGY_ENERGYDETECTOR_H

#include <vector>
#include <opencv2/core.hpp>

class EnergyDetector {
public:
    cv::Point2f armor[4] = {{0, 0},
                            {0, 0},
                            {0, 0},
                            {0, 0}};  //待打击装甲版四个顶点
    cv::Point2f whole_center[4];   //能量机关中心点外接矩形的四个顶点
    bool ifChange = false;  //是否更换了打击目标
    bool firstSee = false;

    /**
     * @brief 识别能量机关
     * @param frame 相机获取的图像
     */
    void detector(const cv::Mat &frame);

    EnergyDetector() {};
private:
    cv::Mat origin;
    int thred = 80; //二值化阈值
    int mediaB = 3; //中值滤波窗口
    float openEle = 1.5;   //开运算尺寸
    float closeEle = 3; //闭运算尺寸
    std::vector<std::vector<cv::Point>> allContours;   //所有轮廓
    std::vector<std::vector<cv::Point>> unContours;    //未打击的所有疑似轮廓
    std::vector<std::vector<std::vector<cv::Point>>> alrContours;   //已打击的所有疑似轮廓
    std::vector<std::vector<cv::Point>> centerContours;   //中心点的所有疑似轮廓
    std::vector<cv::Vec4i> hiera;   //轮廓的层级关系

    cv::RotatedRect energyCenter;   //中心点的旋转矩形
    cv::RotatedRect unStricken;    //未打击装甲版的旋转矩形
    cv::Point2f lastArmor[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    std::vector<cv::RotatedRect> alrStricken;   //已打击装甲版的旋转矩形

    float unStrickenArea;  //未打击装甲版的面积
    int lastAlrStricken = 0;    //上一帧已打击的个数
    int AlrStricken = 0;    //当前帧已打击的个数
    int change_frequency = 0;

    /**
     * @brief 判断是否为未打击的装甲版
     * @param unContours 未打击的所有疑似轮廓
     */
    void judgeUnStricken(std::vector<std::vector<cv::Point>> &unContours);

    /**
     * @brief 判断是否未已打击的装甲版
     * @param alrContours 所有已打击的所有疑似轮廓
     */
    void judgeAlrStricken(std::vector<std::vector<std::vector<cv::Point>>> &alrContours);

    /**
     * @brief 判断是否为中心点
     * @param centerContours 所有中心点的所有疑似轮廓
     */
    void judgeCenStricken(std::vector<std::vector<cv::Point>> &centerContours);

};

#endif //MYENERGY_ENERGYDETECTOR_H
