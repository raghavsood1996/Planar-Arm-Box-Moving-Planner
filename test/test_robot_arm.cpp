#include <gtest/gtest.h>
#include <planar_robot_box_moving/robot_arm.h>
#include <cmath>

// Test fixture class
class RobotArmTest : public ::testing::Test {
protected:
    RobotArm* robot;

    void SetUp() override {
        robot = new RobotArm(1.0, 1.0, 1.0, 0.1);
    }

    void TearDown() override {
        delete robot;
    }
};

TEST_F(RobotArmTest, ForwardKinematicsTest) {
    Eigen::Vector3d joint_angles(M_PI / 4, M_PI / 4, M_PI / 4);
    // Manually calculated expected values
    double x_expected = 0;
    double y_expected = 1 + sqrt(2);
    double alpha_expected = 3 * M_PI / 4;

    Eigen::Vector3d end_effector_pose = robot->forwardKinematics(joint_angles);

    EXPECT_NEAR(end_effector_pose[0], x_expected, 1e-5);
    EXPECT_NEAR(end_effector_pose[1], y_expected, 1e-5);
    EXPECT_NEAR(end_effector_pose[2], alpha_expected, 1e-5);
}

TEST_F(RobotArmTest, InverseKinematicsTest) {
    // Using the updated expected pose from the forward kinematics test
    Eigen::Vector3d end_effector_pose(0, 1 + sqrt(2), 3 * M_PI / 4);
    auto ik_solutions = robot->inverseKinematics(end_effector_pose);

    ASSERT_FALSE(ik_solutions.empty());

    for (const auto& solution : ik_solutions) {
        Eigen::Vector3d computed_pose = robot->forwardKinematics(solution);
        EXPECT_NEAR(computed_pose[0], end_effector_pose[0], 1e-5);
        EXPECT_NEAR(computed_pose[1], end_effector_pose[1], 1e-5);
        EXPECT_NEAR(computed_pose[2], end_effector_pose[2], 1e-5);
    }
}
