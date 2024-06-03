#ifndef PLANAR_ARM_BOX_MOVING_UTILS_HPP
#define PLANAR_ARM_BOX_MOVING_UTILS_HPP

#include "robot_arm.h"
#include "environment.h"
#include "collision_checker.h"

/**
 * @brief Finds a valid collision-free state for the robot given a target pose and a target box.
 * @param robot The robot arm.
 * @param pose The target pose (x, y, theta).
 * @param environment The environment containing the boxes.
 * @param target_box_uid The UID of the target box.
 * @return A pair consisting of a boolean indicating if a valid state was found and the joint angles if valid.
 */
std::pair<bool, Eigen::Vector3d> findValidState(const RobotArm& robot, const Eigen::Vector3d& pose, const Environment& environment, const std::string& target_box_uid);

#endif // PLANAR_ARM_BOX_MOVING_UTILS_HPP