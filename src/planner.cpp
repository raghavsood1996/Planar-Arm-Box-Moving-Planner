#include <planar_robot_box_moving/planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalState.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

Planner::Planner(const RobotArm& robot, const Environment& environment)
    : robot_(robot), environment_(environment) {
    // Create a state space representing the joint angles
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    //set bounds for the joint angles using the robot's joint limits
    bounds.setLow(0, robot.getTheta1Min());
    bounds.setHigh(0, robot.getTheta1Max());
    bounds.setLow(1, robot.getTheta2Min());
    bounds.setHigh(1, robot.getTheta2Max());
    bounds.setLow(2, robot.getTheta3Min());
    bounds.setHigh(2, robot.getTheta3Max());
    space->setBounds(bounds);

    // Create the SimpleSetup object
    setup_ = std::make_shared<og::SimpleSetup>(space);
}

std::vector<Eigen::Vector3d> Planner::planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const std::string& target_box_uid, double clearance_threshold) {
    // Set state validity checker with the target box UID
    setup_->setStateValidityChecker([this, &target_box_uid, &clearance_threshold](const ob::State* state) {
        const auto* state3D = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d joint_angles(state3D->values[0], state3D->values[1], state3D->values[2]);
        CollisionChecker checker(this->robot_, this->environment_);
        checker.setClearanceThreshold(clearance_threshold);
        return !checker.isCollision(joint_angles, target_box_uid);
    });

    // Set the start state
    ob::ScopedState<> start_state(setup_->getSpaceInformation());
    start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
    start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
    start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];

    // Set the goal state
    ob::ScopedState<> goal_state(setup_->getSpaceInformation());
    goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
    goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
    goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];

    // Set the start and goal states
    double goal_threshold = 0.1;
    setup_->setStartState(start_state);
  
    auto goal_region = std::make_shared<ob::GoalState>(setup_->getSpaceInformation());
    goal_region->setThreshold(goal_threshold);
    goal_region->setState(goal_state);
    setup_->setGoal(goal_region);

    // Set the planner
    setup_->setPlanner(std::make_shared<og::RRTConnect>(setup_->getSpaceInformation()));

    // Attempt to solve the planning problem
    ob::PlannerStatus solved = setup_->solve(50.0);

    std::vector<Eigen::Vector3d> path;
    if (solved) {
        og::PathGeometric path_geom = setup_->getSolutionPath();
        // Simplify the path
        og::PathSimplifier simplifier(setup_->getSpaceInformation());
        simplifier.simplifyMax(path_geom);
        path_geom.interpolate();

        // Validate the simplified path
        bool valid = true;
        for (size_t i = 0; i < path_geom.getStateCount(); ++i) {
            const auto* state = path_geom.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d joint_angles(state->values[0], state->values[1], state->values[2]);
            CollisionChecker checker(this->robot_, this->environment_);
            if (checker.isCollision(joint_angles, target_box_uid)) {
                valid = false;
                break;
            }
        }

        if (valid) {
            for (size_t i = 0; i < path_geom.getStateCount(); ++i) {
                const auto* state = path_geom.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                path.emplace_back(state->values[0], state->values[1], state->values[2]);
            }
        } else {
            std::cout << "Simplified path in collision. No valid path found." << std::endl;
        }
    } else {
        std::cout << "No solution found." << std::endl;
    }

    return path;
}