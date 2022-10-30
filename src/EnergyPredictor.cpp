////
//// Created by tioeare on 10/17/22.
////
//#include "EnergyPredictor.h"
//#include "get_time.h"
//
//extern double first_time;
//extern int fps;
//
//void EnergyPre::predict(float &yaw, float &pitch, cv::Point3f armor_pos) {
////    yawNow = yaw - firstYaw;
////    pitchNow = pitch - firstPitch;    //TODO:改成电控传来的当前yaw、pitch
////存数据
//    armorWorldPoint = toWorldPoints(armor_pos);
//    if (armorPoints.size() != size) {
//        armorPoints.push_back(armorWorldPoint);
//        times.push_back((float) (current_time() - first_time));
//        return;
//    }
//    armorPoints.pop_front();
//    times.pop_front();
//    armorPoints.push_back(armorWorldPoint);
//    times.push_back((float) (current_time() - first_time));
//    centerPoint = fixCenter();
//
//    if (speeds.size() != size - gap)
//        if (armorPoints.size() > gap)
//            for (int i = gap; i < size; i++) {
//                speeds.push_back(calSpeed(i));
//            }
//    speeds.pop_front();
//    speeds.push_back(calSpeed(size - 1));
////拟合参数
//    solve();
//    Eigen::Vector3f v1 = {armorPoints[armorPoints.size() - 1].x - centerPoint.x,
//                          armorPoints[armorPoints.size() - 1].y - centerPoint.y,
//                          armorPoints[armorPoints.size() - 1].z - centerPoint.z};
////预测
//    prediction = 0.5;
//    float theta = (abc[0] * sin(abc[1] * (times[times.size() - 1] + prediction / 2)) + abc[2]) * prediction;
//    //TODO:更改为实际需要的预测量
//    Eigen::Matrix3f R;
//    R << cos(theta) + pow(normVec(0), 2) * (1 - cos(theta)),
//            -normVec(2) * sin(theta) + normVec(0) * normVec(1) * (1 - cos(theta)),
//            normVec(1) * sin(theta) + normVec(0) * normVec(2) * (1 - cos(theta)),
//            normVec(2) * sin(theta) + normVec(0) * normVec(1) * (1 - cos(theta)),
//            cos(theta) + pow(normVec(1), 2) * (1 - cos(theta)),
//            -normVec(0) * sin(theta) + normVec(1) * normVec(2) * (1 - cos(theta)),
//            -normVec(1) * sin(theta) + normVec(0) * normVec(2) * (1 - cos(theta)),
//            normVec(0) * sin(theta) + normVec(1) * normVec(2) * (1 - cos(theta)),
//            cos(theta) + pow(normVec(2), 2) * (1 - cos(theta));
//    Eigen::Vector3f v2 = R * v1;
//    v2[0] += centerPoint.x;
//    v2[1] += centerPoint.y;
//    v2[2] += centerPoint.z;
//    cv::Point3f toStrick = toWorldPoints(v2);
//    float distance = sqrt(pow(toStrick.x, 2) + pow(toStrick.y, 2) + pow(toStrick.z, 2));
//    std::cout << "******************************" << std::endl;
//    std::cout << "armor_pos: " << armor_pos.x << "\t" << armor_pos.y << "\t" << armor_pos.z << std::endl;
//    std::cout << "centerPoint: " << centerPoint.x << "\t" << centerPoint.y << "\t" << centerPoint.z << std::endl;
////    std::cout << "before: yaw: " << yaw << "\tpitch: " << pitch << std::endl;
//    yaw = atan2(toStrick.x, toStrick.z) * 180 / M_PI;
//    pitch = asin(toStrick.y / distance) * 180 / M_PI;
////    std::cout << "after: yaw: " << yaw << " pitch: " << pitch << std::endl;
////    std::cout << "before speed: " << speeds[speeds.size() - 1] << std::endl;
////    std::cout << "after speed: " << (abc[0] * sin(abc[1] * (times[times.size() - 1] + (1 / (float) fps))) + abc[2])
////              << std::endl;
////    std::cout << "abc: " << abc[0] << " & " << abc[1] << " & " << abc[2] << std::endl;
//}
//
//cv::Point3f EnergyPre::toWorldPoints(cv::Point3f armor) {
//    Eigen::Matrix3f R;
//    R << cos(yawNow) + sin(yawNow), sin(yawNow) * sin(pitchNow), sin(yawNow) * cos(pitchNow),
//            0, cos(pitchNow), -sin(pitchNow),
//            -sin(yawNow), cos(yawNow) * sin(pitchNow), cos(yawNow) * cos(pitchNow);
//    Eigen::Vector3f pre = {armor.x, armor.y, armor.z};
//    Eigen::Vector3f after = R.inverse() * pre;
//    return cv::Point3f{after(0), after(1), after(2)};
//}
//
//cv::Point3f EnergyPre::toWorldPoints(Eigen::Vector3f armor) {
//    Eigen::Matrix3f R;
//    R << cos(yawNow) + sin(yawNow), sin(yawNow) * sin(pitchNow), sin(yawNow) * cos(pitchNow),
//            0, cos(pitchNow), -sin(pitchNow),
//            -sin(yawNow), cos(yawNow) * sin(pitchNow), cos(yawNow) * cos(pitchNow);
//    Eigen::Vector3f after = R * armor;
//    return cv::Point3f{after(0), after(1), after(2)};
//}
//
//cv::Point3f EnergyPre::fixCenter() {
//    Eigen::MatrixXf B(size - 1, 3);
//    Eigen::VectorXf L2(size - 1);
//    Eigen::VectorXf L1(size - 1);
//    L1.setOnes();
//    Eigen::MatrixXf M(size - 1, 3);
//    for (int i = 0; i < size - 1; i++) {
//        B(i, 0) = armorPoints[i + 1].x - armorPoints[i].x;
//        B(i, 1) = armorPoints[i + 1].y - armorPoints[i].y;
//        B(i, 2) = armorPoints[i + 1].z - armorPoints[i].z;
//        M(i, 0) = armorPoints[i].x;
//        M(i, 1) = armorPoints[i].y;
//        M(i, 2) = armorPoints[i].z;
//        L2(i) = 0.5 * (pow(armorPoints[i + 1].x, 2) + pow(armorPoints[i + 1].y, 2) + pow(armorPoints[i + 1].z, 2) -
//                       pow(armorPoints[i].x, 2) - pow(armorPoints[i].y, 2) - pow(armorPoints[i].z, 2));
//    }
//
//    Eigen::VectorXf A = ((M.transpose() * M).inverse()) * M.transpose() * L1;
//    Eigen::MatrixXf H(4, 3);
//    H << B.transpose() * B, A.transpose();
//    Eigen::Vector4f D;
//    D << B.transpose() * L2, 1;
//    Eigen::VectorXf C = ((H.transpose() * H).inverse()) * H.transpose() * D;
//    return cv::Point3f{C(0), C(1), C(2)};
//}
//
//float EnergyPre::calSpeed(int i) {
//    Eigen::Vector3f pre = {armorPoints[i - gap].x, armorPoints[i - gap].y, armorPoints[i - gap].z};
//    Eigen::Vector3f now = {armorPoints[i].x, armorPoints[i].y, armorPoints[i].z};
//    float speed = (calAngle(pre, now) / (times[i] - times[i - gap]));
//    if (speed > 2.5) return speed = 2.5;
//    else if (speed < -2.5) return speed = -2.5;
//    else
//        return speed;
//}
//
//float EnergyPre::calAngle(Eigen::Vector3f point1, Eigen::Vector3f point2) {
//    Eigen::Vector3f p = {centerPoint.x, centerPoint.y, centerPoint.z};
//    Eigen::Vector3f v1 = point1 - p;
//    Eigen::Vector3f v2 = point2 - p;
//    normVec = v1.cross(v2);
//    float radian_angle = atan2(normVec.norm(), v1.transpose() * v2);
//    if (v1.cross(v2).z() < 0) {
//        radian_angle = 2 * M_PI - radian_angle;
//    }
//    return radian_angle;
//}
//
//void EnergyPre::solve() {
//    abc[0] = 0.785;
//    abc[1] = 1.884;
//    abc[2] = 1.305;
//    ceres::Problem problem;
//    for (int i = 0; i < speeds.size() - 1; i++) {
//        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
//                new CURVE_FITTING_COST((double) times[i], (double) speeds[i])), NULL, abc
//        );
//    }
//    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);
//}