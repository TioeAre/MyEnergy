//
// Created by tioeare on 10/14/22.
//
#include <iostream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "EnergyDetector.h"

extern bool IF_DEBUG;
extern int fps;

void restore_Rect(cv::Point2f pts[4], cv::Point2f points[4]);   //还原带打击装甲版的外接矩形
float calRectArea(cv::RotatedRect rect);   //计算旋转矩形的面积
float dis(cv::Point2f pts1, cv::Point2f pts2);  //计算两点距离
float calMaxEdge(cv::RotatedRect rect); //计算矩形最长边
void drawRect(cv::RotatedRect rect, const cv::Mat &image, const cv::Scalar scal);

void EnergyDetector::detector(const cv::Mat &frame) {
    ifChange = false;
    origin = frame.clone();
    cv::Mat trans, thre, final, mask, masked;
    std::vector<cv::Mat> frmChannels(3); //bgr
    cv::split(origin, frmChannels);  //通道分离
    cv::subtract(frmChannels[2], frmChannels[0], trans);    //通道相减
    cv::subtract(frmChannels[2], frmChannels[1], mask);
    cv::medianBlur(trans, trans, mediaB);   //中值滤波
//    cv::bilateralFilter(trans, thre, 5, 10, 3);   //双边滤波
    cv::threshold(trans, thre, thred, 255, cv::THRESH_BINARY);  //二值化
//    cv::threshold(mask, mask, 20, 255, cv::THRESH_BINARY);
    thre.copyTo(masked, mask);
    //开操作去除噪声，闭操作使轮廓连起来
    cv::Mat struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(openEle, openEle), cv::Point(-1, -1));
    cv::morphologyEx(masked, final, cv::MORPH_OPEN, struElement);
    struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeEle, closeEle), cv::Point(-1, -1));
    cv::morphologyEx(final, final, cv::MORPH_CLOSE, struElement);
    //初始化所有容器，删掉上一帧残留
    allContours.resize(0), unContours.resize(0), alrContours.resize(0), centerContours.resize(0);
    alrStricken.resize(0);

    cv::findContours(final, allContours, hiera, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);   //查找轮廓
    if (hiera.size() < 3) return;    //判断是否识别到，识别到时至少有三个轮廓
    int i = 0;
    for (const auto &itcHier: hiera) {
        if (itcHier[2] == -1 && itcHier[3] == -1) {
            centerContours.push_back((allContours[i]));
        }//没有子轮廓和父轮廓可能是中心点的轮廓
        else if (itcHier[2] > -1 && hiera[itcHier[2]][0] == -1 && itcHier[3] == -1) {
            unContours.push_back(allContours[itcHier[2]]);
        }//有且只有一个子轮廓，且没有父轮廓，可能是未打击的
        else if (itcHier[2] > -1 && hiera[itcHier[2]][0] > -1 && hiera[hiera[itcHier[2]][0]][0] > -1
                 && hiera[hiera[hiera[itcHier[2]][0]][0]][0] == -1 && itcHier[3] == -1) {
            std::vector<std::vector<cv::Point>> threeCon = {allContours[itcHier[2]], allContours[hiera[itcHier[2]][0]],
                                                            allContours[hiera[hiera[itcHier[2]][0]][0]]};
            alrContours.push_back(threeCon);
        }//有三个子轮廓的是未打击的，且没有父轮廓，可能是个已打击的
        i++;
    }
    judgeUnStricken(unContours);
    judgeAlrStricken(alrContours);
    judgeCenStricken(centerContours);
    cv::Point2f last_armor_center;
    last_armor_center.x = 0.5 * (lastArmor[0].x + lastArmor[2].x);
    last_armor_center.y = 0.5 * (lastArmor[0].y + lastArmor[2].y);
    cv::Point2f armor_center;
    armor_center.x = 0.5 * (armor[0].x + armor[2].x);
    armor_center.y = 0.5 * (armor[0].y + armor[2].y);
    if (dis(armor_center, last_armor_center) > dis(armor[0], armor[3])) ifChange = true;    //TODO:调参
    //绘图
    if (IF_DEBUG) {
        drawRect(energyCenter, origin, cv::Scalar(255, 255, 0));
        drawRect(unStricken, origin, cv::Scalar(0, 255, 0));
        if (!alrStricken.empty())
            for (int i = 0; i < alrStricken.size(); i++) drawRect(alrStricken[i], origin, cv::Scalar(255, 100, 155));
        for (int i = 0; i < 4; i++) {
            cv::putText(origin, std::to_string(i + 1), cv::Point(armor[i].x, armor[i].y), cv::FONT_HERSHEY_SIMPLEX,
                        0.45,
                        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        cv::putText(origin, std::to_string(fps), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.95,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::namedWindow("origin", cv::WINDOW_NORMAL);
        cv::imshow("origin", origin);
    }
}

void EnergyDetector::judgeUnStricken(std::vector<std::vector<cv::Point>> &unContours) {
    cv::RotatedRect unJudge;
    cv::Point2f pts[4];
    for (const auto &unContour: unContours) {
        unJudge = cv::minAreaRect(unContour);
        unJudge.points(pts);
        cv::Point2f pts1[4];
        restore_Rect(pts, pts1);
        float width = dis(pts1[0], pts1[2]);
        float height = dis(pts1[0], pts1[1]);
        unStrickenArea = width * height;
        if (0.4 < (height / width) && (height / width) < 0.8) {    //判断长宽比
            for (int i = 0; i < 4; i++) {
                lastArmor[i].x = armor[i].x;
                lastArmor[i].y = armor[i].y;
                armor[i].x = pts1[i].x;
                armor[i].y = pts1[i].y;
            }
            unStricken = unJudge;
            firstSee = true;
        }
    }
}

//已打击装甲板alrContours是一个三层的vector，第一层是每一个已打击的扇叶，第二层是已打击扇叶的三个子轮廓，第三层是每个轮廓的点
void EnergyDetector::judgeAlrStricken(std::vector<std::vector<std::vector<cv::Point>>> &alrContours) {
    cv::RotatedRect unJudge;
    std::vector<std::pair<int, cv::RotatedRect>> thrCon;
    AlrStricken = 0;
    for (auto &&alrContour: alrContours) {
        thrCon.resize(0);
        if (alrContour.size() == 3) {   //三个子轮廓判断，防止传进来的是空值导致程序意外退出
            for (int i = 0; i < alrContour.size(); i++) {
                unJudge = cv::minAreaRect(alrContour[i]);
                thrCon.push_back({i, unJudge});
            }
            std::sort(thrCon.begin(), thrCon.end(),
                      [](std::pair<int, cv::RotatedRect> rect1, std::pair<int, cv::RotatedRect> rect2) {
                          return calMaxEdge(rect1.second) < calMaxEdge(rect2.second);
                      });   //对矩形边长从小到大排序
            if (0.5 < calRectArea(thrCon[0].second) / unStrickenArea &&
                calRectArea(thrCon[0].second) / unStrickenArea < 2) {
                alrStricken.push_back(thrCon[0].second);
                AlrStricken++;
            }
        }
    }
//    //判读打击目标是否发生变化
//    if ((lastAlrStricken != AlrStricken && AlrStricken == 0) || AlrStricken - lastAlrStricken == 1) {
//        ifChange = true;
//        if (IF_DEBUG) {
//            change_frequency++;
//        }
//    }
    lastAlrStricken = AlrStricken;
}

void EnergyDetector::judgeCenStricken(std::vector<std::vector<cv::Point>> &centerContours) {
    cv::RotatedRect unJudge;
    for (const auto &centerContour: centerContours) {
        unJudge = cv::minAreaRect(centerContour);
        cv::Point2f armorCenter = unStricken.center;
        cv::Point2f centerPoint = unJudge.center;
        float angle = abs(atan2(centerPoint.y - armorCenter.y, centerPoint.x - armorCenter.x)) * 180 / M_PI;
        //判断疑似中心点与待打击装甲板的角度值，长度比和面积比来确定中心点
        if ((abs(angle - unStricken.angle) < 20 || abs(90 - angle + unStricken.angle) < 20) &&
            0.1 < calRectArea(unJudge) / calRectArea(unStricken) &&
            calRectArea(unJudge) / calRectArea(unStricken) < 0.5 &&
            dis(unJudge.center, unStricken.center) / dis(armor[0], armor[1]) < 8) {
            energyCenter = unJudge;
            energyCenter.points(whole_center);
        }
    }
}

void restore_Rect(cv::Point2f pts[4], cv::Point2f points[4]) {
    std::vector<std::pair<int, float>>
            lengths = {{1, dis(pts[0], pts[1])},
                       {2, dis(pts[0], pts[2])},
                       {3, dis(pts[0], pts[3])}};
    std::sort(lengths.begin(), lengths.end(), [](std::pair<int, float> length1, std::pair<int, float> length2) {
        return length1.second < length2.second;
    }); //从小到大排序
    if (lengths[0].first == 1) {
        points[0] = pts[0];
        points[1] = pts[lengths[0].first];
        points[2] = pts[lengths[2].first];
        points[3] = pts[lengths[1].first];
    } else {
        points[0] = pts[lengths[0].first];
        points[1] = pts[0];
        points[2] = pts[lengths[1].first];
        points[3] = pts[lengths[2].first];
    }
}

float dis(cv::Point2f pts1, cv::Point2f pts2) {
    return std::sqrt(std::pow(pts1.x - pts2.x, 2) + std::pow(pts1.y - pts2.y, 2));
}

float calRectArea(cv::RotatedRect rect) {
    cv::Point2f pts[4];
    rect.points(pts);
    return dis(pts[0], pts[1]) * dis(pts[0], pts[2]);
}

float calMaxEdge(cv::RotatedRect rect) {
    cv::Point2f pts[4];
    rect.points(pts);
    std::vector<std::pair<int, float>>
            lengths = {{1, dis(pts[0], pts[1])},
                       {2, dis(pts[0], pts[2])},
                       {3, dis(pts[0], pts[3])}};
    std::sort(lengths.begin(), lengths.end(), [](std::pair<int, float> length1, std::pair<int, float> length2) {
        return length1.second > length2.second;
    });
    return lengths[0].second;
}

void drawRect(cv::RotatedRect rect, const cv::Mat &image, const cv::Scalar scal) {
    //获取旋转矩形的四个顶点
    cv::Point2f *vertices = new cv::Point2f[4];
    rect.points(vertices);
    //逐条边绘制
    for (int j = 0; j < 4; j++) {
        cv::line(image, vertices[j], vertices[(j + 1) % 4], scal);
    }
}