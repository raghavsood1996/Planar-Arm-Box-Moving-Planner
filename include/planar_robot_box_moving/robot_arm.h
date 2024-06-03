#ifndef PLANAR_ARM_BOX_MOVING_ROBOT_ARM_HPP
#define PLANAR_ARM_BOX_MOVING_ROBOT_ARM_HPP

#include <Eigen/Dense>
#include <vector>

/**
 * @class RobotArm
 * @brief A class to represent a 3DOF planar robot arm and perform forward and inverse kinematics.
 */
class RobotArm {
public:
    /**
     * @brief Constructor to initialize the robot arm with given link lengths and width.
     * @param L1 Length of the first link.
     * @param L2 Length of the second link.
     * @param L3 Length of the third link.
     * @param w Width of the links.
     */
    RobotArm(double L1, double L2, double L3, double w);

    /**
     * @brief Computes the forward kinematics for the given joint angles.
     * @param joint_angles A 3x1 vector of joint angles (theta1, theta2, theta3).
     * @return A 3x1 vector representing the end-effector position (x, y, alpha).
     */
    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& joint_angles) const;

    /**
     * @brief Computes the inverse kinematics for the given end-effector pose.
     * @param end_effector_pose A 3x1 vector representing the desired end-effector position (x, y, alpha).
     * @return A vector of 3x1 vectors, each representing a possible solution for the joint angles (theta1, theta2, theta3).
     */
    std::vector<Eigen::Vector3d> inverseKinematics(const Eigen::Vector3d& end_effector_pose) const;

    /**
     * @brief Getter for the length of the first link.
     * @return Length of the first link.
     */
    double getL1() const;

    /**
     * @brief Getter for the length of the second link.
     * @return Length of the second link.
     */
    double getL2() const;

    /**
     * @brief Getter for the length of the third link.
     * @return Length of the third link.
     */
    double getL3() const;

    /**
     * @brief Getter for the width of the links.
     * @return Width of the links.
     */
    double getW() const;

    //declar a getter for the joint limits
    double getTheta1Min() const;
    double getTheta1Max() const;
    double getTheta2Min() const;
    double getTheta2Max() const;
    double getTheta3Min() const;
    double getTheta3Max() const;

private:
    double L1_; ///< Length of the first link.
    double L2_; ///< Length of the second link.
    double L3_; ///< Length of the third link.
    double w_;  ///< Width of the links.

    // Joint limits
    double theta1_min_, theta1_max_;
    double theta2_min_, theta2_max_;
    double theta3_min_, theta3_max_;

    /**
     * @brief Helper function to solve inverse kinematics for the wrist position.
     * @param wx Wrist x-coordinate.
     * @param wy Wrist y-coordinate.
     * @param alpha End-effector orientation.
     * @param solutions Vector to store the solutions for joint angles.
     */
    void solveInverseKinematics(double wx, double wy, double alpha, std::vector<Eigen::Vector3d>& solutions) const;
};

#endif // PLANAR_ARM_BOX_MOVING_ROBOT_ARM_HPP
