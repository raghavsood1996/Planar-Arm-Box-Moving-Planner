#include <planar_robot_box_moving/robot_arm.h>
#include <cmath>
#include <iostream>

RobotArm::RobotArm(double L1, double L2, double L3, double w)
    : L1_(L1), L2_(L2), L3_(L3), w_(w),
      theta1_min_(0), theta1_max_(M_PI),
      theta2_min_(-M_PI / 2), theta2_max_(M_PI / 2),
      theta3_min_(-M_PI / 2), theta3_max_(M_PI / 2) {}

Eigen::Vector3d RobotArm::forwardKinematics(const Eigen::Vector3d& joint_angles) const {
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];

    double x = L1_ * cos(theta1) + L2_ * cos(theta1 + theta2) + L3_ * cos(theta1 + theta2 + theta3);
    double y = L1_ * sin(theta1) + L2_ * sin(theta1 + theta2) + L3_ * sin(theta1 + theta2 + theta3);
    double alpha = theta1 + theta2 + theta3;

    return Eigen::Vector3d(x, y, alpha);
}

void RobotArm::solveInverseKinematics(double wx, double wy, double alpha, std::vector<Eigen::Vector3d>& solutions) const {
    double D = (wx * wx + wy * wy - L1_ * L1_ - L2_ * L2_) / (2 * L1_ * L2_);
    if (D <= -1.000000001 || D >= 1.0000000001) {
        std::cout << "D out of range: " << D << std::endl;
        return; // No solution
    }

    double theta2_1 = atan2(sqrt(1 - D * D), D);
    double theta2_2 = atan2(-sqrt(1 - D * D), D);

    double theta1_1 = atan2(wy, wx) - atan2(L2_ * sin(theta2_1), L1_ + L2_ * cos(theta2_1));
    double theta1_2 = atan2(wy, wx) - atan2(L2_ * sin(theta2_2), L1_ + L2_ * cos(theta2_2));

    double theta3_1 = alpha - theta1_1 - theta2_1;
    double theta3_2 = alpha - theta1_2 - theta2_2;

    // Check joint limits
    if (theta1_1 >= theta1_min_ && theta1_1 <= theta1_max_ &&
        theta2_1 >= theta2_min_ && theta2_1 <= theta2_max_ &&
        theta3_1 >= theta3_min_ && theta3_1 <= theta3_max_) {
        solutions.push_back(Eigen::Vector3d(theta1_1, theta2_1, theta3_1));
    }
    if (theta1_2 >= theta1_min_ && theta1_2 <= theta1_max_ &&
        theta2_2 >= theta2_min_ && theta2_2 <= theta2_max_ &&
        theta3_2 >= theta3_min_ && theta3_2 <= theta3_max_) {
        solutions.push_back(Eigen::Vector3d(theta1_2, theta2_2, theta3_2));
    }
}

std::vector<Eigen::Vector3d> RobotArm::inverseKinematics(const Eigen::Vector3d& end_effector_pose) const {
    double x = end_effector_pose[0];
    double y = end_effector_pose[1];
    double alpha = end_effector_pose[2];

    std::vector<Eigen::Vector3d> solutions;

    double wx = x - L3_ * cos(alpha);
    double wy = y - L3_ * sin(alpha);

    // Debug output for wrist position
    std::cout << "Wrist position: (" << wx << ", " << wy << ")" << std::endl;

    solveInverseKinematics(wx, wy, alpha, solutions);

    return solutions;
}

double RobotArm::getL1() const {
    return L1_;
}

double RobotArm::getL2() const {
    return L2_;
}

double RobotArm::getL3() const {
    return L3_;
}

double RobotArm::getW() const {
    return w_;
}

double RobotArm::getTheta1Min() const {
    return theta1_min_;
}

double RobotArm::getTheta1Max() const {
    return theta1_max_;
}

double RobotArm::getTheta2Min() const {
    return theta2_min_;
}

double RobotArm::getTheta2Max() const {
    return theta2_max_;
}

double RobotArm::getTheta3Min() const {
    return theta3_min_;
}

double RobotArm::getTheta3Max() const {
    return theta3_max_;
}

