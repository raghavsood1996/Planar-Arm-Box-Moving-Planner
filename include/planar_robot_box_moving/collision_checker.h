#ifndef PLANAR_ARM_BOX_MOVING_COLLISION_CHECKER_HPP
#define PLANAR_ARM_BOX_MOVING_COLLISION_CHECKER_HPP

#include <planar_robot_box_moving/robot_arm.h>
#include <planar_robot_box_moving/environment.h>

/**
 * @class CollisionChecker
 * @brief A class to check for collisions between the robot arm, target box, and surrounding boxes.
 */
class CollisionChecker {
public:
    /**
     * @brief Constructor to initialize the collision checker with the robot and environment.
     * @param robot The robot arm.
     * @param environment The environment containing the boxes.
     */
    CollisionChecker(const RobotArm& robot, const Environment& environment);

    /**
     * @brief Check if the robot arm or target box is in collision with any surrounding boxes.
     * @param joint_angles The joint angles of the robot arm.
     * @param box_uid The UID of the box attached to the end effector.
     * @return True if there is a collision, false otherwise.
     */
    bool isCollision(const Eigen::Vector3d& joint_angles, const std::string& box_uid) const;

    /**
     * @brief Set the clearance threshold for collision checking.
     * @param clearance_threshold The clearance threshold.
     */
    void setClearanceThreshold(double clearance_threshold);

private:
    const RobotArm& robot_;
    const Environment& environment_;
    double clearance_threshold_ = 1e-5;

    /**
     * @brief Check if two boxes are colliding.
     * @param box1 The first box.
     * @param box2 The second box.
     * @return True if the boxes are colliding, false otherwise.
     */
    bool areBoxesColliding(const Box& box1, const Box& box2, double clearance_threshold) const;

    /**
     * @brief Check if the robot arm is in collision with any surrounding boxes.
     * @param joint_angles The joint angles of the robot arm.
     * @param target_box_uid The UID of the box attached to the end effector.
     * @return True if there is a collision, false otherwise.
     */
    bool isRobotArmInCollision(const Eigen::Vector3d& joint_angles, const std::string& target_box_uid) const;

    /**
     * @brief Check if the target box is in collision with any surrounding boxes.
     * @param target_box The box attached to the end effector.
     * @return True if there is a collision, false otherwise.
     */
    bool isTargetBoxInCollision(const Box& target_box) const;

    /**
     * @brief Get the pose of the box attached to the end effector.
     * @param joint_angles The joint angles of the robot arm.
     * @param box The box attached to the end effector.
     * @return The pose of the box (x, y, alpha).
     */
    Eigen::Vector3d getAttachedBoxPose(const Eigen::Vector3d& joint_angles, const Box& box) const;
};

#endif // PLANAR_ARM_BOX_MOVING_COLLISION_CHECKER_HPP
