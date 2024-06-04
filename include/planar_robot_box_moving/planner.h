#ifndef PLANAR_ARM_BOX_MOVING_PLANNER_HPP
#define PLANAR_ARM_BOX_MOVING_PLANNER_HPP

#include <planar_robot_box_moving/robot_arm.h>
#include <planar_robot_box_moving/environment.h>
#include <planar_robot_box_moving/collision_checker.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @class Planner
 * @brief A class to plan paths for the robot arm.
 */
class Planner {
public:
    /**
     * @brief Constructor to initialize the planner.
     * @param robot The robot arm.
     * @param environment The environment containing the boxes.
     */
    Planner(const RobotArm& robot, const Environment& environment);

    /**
     * @brief Plan a path from the start state to the goal state.
     * @param start The start joint angles.
     * @param goal The goal joint angles.
     * @param target_box_uid The UID of the target box.
     * @param clearance_threshold The clearance threshold for collision checking.
     * @return A vector of joint angles representing the planned path.
     */
    std::vector<Eigen::Vector3d> planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const std::string& target_box_uid, double clearance_threshold);

private:
    const RobotArm& robot_;
    const Environment& environment_;
    std::shared_ptr<ompl::geometric::SimpleSetup> setup_;
};

#endif // PLANAR_ARM_BOX_MOVING_PLANNER_HPP
