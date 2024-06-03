#include <planar_robot_box_moving/utils.h>
#include <iostream>

std::pair<bool, Eigen::Vector3d> findValidState(const RobotArm& robot, const Eigen::Vector3d& pose, const Environment& environment, const std::string& target_box_uid) {
    CollisionChecker collision_checker(robot, environment);
    std::vector<Eigen::Vector3d> joint_angles = robot.inverseKinematics(pose);

    if (joint_angles.empty()) {
        std::cout << "No inverse kinematics solution found for pose: " << pose.transpose() << std::endl;
        return {false, Eigen::Vector3d()};
    }

    for (const auto& angles : joint_angles) {
        if (!collision_checker.isCollision(angles, target_box_uid)) {
            std::cout << "Found collision-free joint angles: " << angles.transpose() << std::endl;
            return {true, angles};
        }
    }

    std::cout << "No collision-free joint angles found for pose: " << pose.transpose() << std::endl;
    return {false, Eigen::Vector3d()};
}
