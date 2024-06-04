#ifndef PLANAR_ARM_BOX_MOVING_UTILS_HPP
#define PLANAR_ARM_BOX_MOVING_UTILS_HPP

#include "robot_arm.h"
#include "environment.h"
#include "collision_checker.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>
#include <array>
#include <string>

/**
 * @brief Finds a valid collision-free state for the robot given a target pose and a target box.
 * @param robot The robot arm.
 * @param pose The target pose (x, y, theta).
 * @param environment The environment containing the boxes.
 * @param target_box_uid The UID of the target box.
 * @return A pair consisting of a boolean indicating if a valid state was found and the joint angles if valid.
 */
std::pair<bool, Eigen::Vector3d> findValidState(const RobotArm& robot, const Eigen::Vector3d& pose, const Environment& environment, const std::string& target_box_uid);

/**
 * @brief Creates a link (cylinder) in VTK.
 * @param length Length of the link.
 * @param width Width of the link.
 * @param start_position Position of the start of the link.
 * @param angle Angle of the link.
 * @param color Color of the link (RGB array).
 * @return A vtkSmartPointer to the actor representing the link.
*/
vtkSmartPointer<vtkActor> createLink(double length, double width, const Eigen::Vector3d& start_position, double angle, const std::array<double, 3>& color);

/**
 * @brief Plots the robot arm and environment using VTK.
 * @param robot The robot arm.
 * @param path The path to plot.
 * @param environment The environment containing the boxes.
 * @param target_box_uid The UID of the target box.
*/
void plotRobotArmAndEnvironment(const RobotArm& robot, const std::vector<Eigen::Vector3d>& path, const Environment& environment, const std::string& target_box_uid);

/**
 * @brief loads the environment from a file.
 * @param filename The name of the file to load.
 * @param environment The environment to load the boxes into.
*/
void loadEnvironment(const std::string& filename, Environment& environment);

/**
 * @brief loads test cases from a file.
 * @param filename The name of the file to load.
 * @param test_cases The vector to load the test cases into.
*/
void loadTestCases(const std::string& filename, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::string, double>>& test_cases);

#endif // PLANAR_ARM_BOX_MOVING_UTILS_HPP