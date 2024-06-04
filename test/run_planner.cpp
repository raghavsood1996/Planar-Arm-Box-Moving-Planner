#include <planar_robot_box_moving/robot_arm.h>
#include <planar_robot_box_moving/environment.h>
#include <planar_robot_box_moving/collision_checker.h>
#include <planar_robot_box_moving/utils.h>
#include <planar_robot_box_moving/planner.h>


#include <iostream>
#include <vector>


int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <environment.yaml> <test_cases.yaml>" << std::endl;
        return 1;
    }

    std::string environment_file = argv[1];
    std::string test_cases_file = argv[2];

    RobotArm robot(1.0, 1.0, 1.0, 0.1);

    // Define the environment with some boxes and a default color
    std::array<double, 3> box_color = {0.8, 0.1, 0.1}; // Red color for boxes
    Environment environment(box_color);

    // Load environment from YAML file
    loadEnvironment(environment_file, environment);

    // Initialize the collision checker
    CollisionChecker collision_checker(robot, environment);

    // Load test cases from YAML file
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::string, double>> test_cases;
    loadTestCases(test_cases_file, test_cases);

    for (const auto& [start_pose, goal_pose, target_box_uid, clearance_threshold] : test_cases) {
        std::cout << "Testing with start pose: " << start_pose.transpose() << ", goal pose: " << goal_pose.transpose() << ", target box: " << target_box_uid << std::endl;

        // Find and check valid state for the start pose
        auto [start_found, start_joint_angles] = findValidState(robot, start_pose, environment, target_box_uid);
        if (!start_found) {
            std::cout << "No valid state found for the start pose." << std::endl;
            continue;
        }

        // Find and check valid state for the goal pose
        auto [goal_found, goal_joint_angles] = findValidState(robot, goal_pose, environment, target_box_uid);
        if (!goal_found) {
            std::cout << "No valid state found for the goal pose." << std::endl;
            continue;
        }

        
        Planner planner(robot, environment);
        std::vector<Eigen::Vector3d> path = planner.planPath(start_joint_angles, goal_joint_angles, target_box_uid, clearance_threshold);
        
        if (path.empty()) {
            std::cout << "No valid path found." << std::endl;
            continue;
        }

        // Visualize the planned path
        plotRobotArmAndEnvironment(robot, path, environment, target_box_uid);
    }

    return 0;
}