#include <planar_robot_box_moving/collision_checker.h>
#include <cmath>

CollisionChecker::CollisionChecker(const RobotArm& robot, const Environment& environment)
    : robot_(robot), environment_(environment) {}

bool CollisionChecker::isCollision(const Eigen::Vector3d& joint_angles, const std::string& box_uid) const {
    // Check if the robot arm is in collision
    if (isRobotArmInCollision(joint_angles, box_uid)) {
        return true;
    }

    // Get the box attached to the end effector
    Box target_box("", 0, 0, 0, 0, 0); // Initialize with default values
    for (const auto& box : environment_.getBoxes()) {
        if (box.getUID() == box_uid) {
            target_box = box;
            break;
        }
    }

    // Calculate the pose of the target box attached to the end effector
    Eigen::Vector3d target_box_pose = getAttachedBoxPose(joint_angles, target_box);
    Box attached_box(target_box.getUID(), target_box_pose[0], target_box_pose[1], target_box_pose[2], target_box.getSize()[0], target_box.getSize()[1]);

    // Check if the target box is in collision
    if (isTargetBoxInCollision(attached_box)) {
        return true;
    }

    return false;
}

bool CollisionChecker::areBoxesColliding(const Box& box1, const Box& box2, double clearance_threshold) const {

    Eigen::Vector3d pos1 = box1.getPosition();
    Eigen::Vector3d pos2 = box2.getPosition();
    Eigen::Vector2d size1 = box1.getSize();
    Eigen::Vector2d size2 = box2.getSize();

    Eigen::Vector2d halfSize1 = size1 / 2.0;
    Eigen::Vector2d halfSize2 = size2 / 2.0;

    double alpha_1 = pos1[2];
    double alpha_2 = pos2[2];

    Eigen::Matrix2d rotationMatrix1;
    rotationMatrix1 << cos(alpha_1), -sin(alpha_1),
          sin(alpha_1), cos(alpha_1);
    
    Eigen::Matrix2d rotationMatrix2;
    rotationMatrix2 << cos(alpha_2), -sin(alpha_2),
          sin(alpha_2), cos(alpha_2);

    Eigen::Vector2d corners1[4], corners2[4];

    // Calculate the corners of the first box
    corners1[0] = pos1.head<2>() + rotationMatrix1 * Eigen::Vector2d(-halfSize1.x(), -halfSize1.y());
    corners1[1] = pos1.head<2>() + rotationMatrix1 * Eigen::Vector2d(halfSize1.x(), -halfSize1.y());
    corners1[2] = pos1.head<2>() + rotationMatrix1 * Eigen::Vector2d(halfSize1.x(), halfSize1.y());
    corners1[3] = pos1.head<2>() + rotationMatrix1 * Eigen::Vector2d(-halfSize1.x(), halfSize1.y());

    // Calculate the corners of the second box
    corners2[0] = pos2.head<2>() + rotationMatrix2 * Eigen::Vector2d(-halfSize2.x(), -halfSize2.y());
    corners2[1] = pos2.head<2>() + rotationMatrix2 * Eigen::Vector2d(halfSize2.x(), -halfSize2.y());
    corners2[2] = pos2.head<2>() + rotationMatrix2 * Eigen::Vector2d(halfSize2.x(), halfSize2.y());
    corners2[3] = pos2.head<2>() + rotationMatrix2 * Eigen::Vector2d(-halfSize2.x(), halfSize2.y());

    // SAT helper functions
    auto getProjection = [](const Eigen::Vector2d& axis, const Eigen::Vector2d corners[4], double threshold) -> Eigen::Vector2d {
        double min = axis.dot(corners[0]);
        double max = min;
        for (int i = 1; i < 4; ++i) {
            double projection = axis.dot(corners[i]);
            if (projection < min) {
                min = projection;
            } else if (projection > max) {
                max = projection;
            }
        }
        // Adjust projections by the threshold value
        return Eigen::Vector2d(min - threshold, max + threshold);
    };

    auto overlap = [](const Eigen::Vector2d& proj1, const Eigen::Vector2d& proj2) -> bool {
        return !(proj1[1] < proj2[0] || proj2[1] < proj1[0]);
    };

    // Axes to test
    Eigen::Vector2d axes[4] = {
        (corners1[1] - corners1[0]).normalized(),
        (corners1[3] - corners1[0]).normalized(),
        (corners2[1] - corners2[0]).normalized(),
        (corners2[3] - corners2[0]).normalized()
    };

    // Check for overlap on all axes
    for (const auto& axis : axes) {
        Eigen::Vector2d proj1 = getProjection(axis, corners1, clearance_threshold);
        Eigen::Vector2d proj2 = getProjection(axis, corners2, clearance_threshold);
        if (!overlap(proj1, proj2)) {
            return false; // Separating axis found, no collision
        }
    }

    return true; // No separating axis found, collision detected
}

bool CollisionChecker::isRobotArmInCollision(const Eigen::Vector3d& joint_angles, const std::string& target_box_uid) const {
    // Get the positions of the robot arm links
    Eigen::Vector3d p0(0, 0, 0);
    double angle1 = joint_angles[0];
    Eigen::Vector3d p1 = p0 + Eigen::Vector3d(robot_.getL1() * cos(angle1), robot_.getL1() * sin(angle1), 0);

    double angle2 = angle1 + joint_angles[1];
    Eigen::Vector3d p2 = p1 + Eigen::Vector3d(robot_.getL2() * cos(angle2), robot_.getL2() * sin(angle2), 0);

    double angle3 = angle2 + joint_angles[2];
    Eigen::Vector3d p3 = p2 + Eigen::Vector3d(robot_.getL3() * cos(angle3), robot_.getL3() * sin(angle3), 0);

    // Adjust bounding boxes to better reflect link dimensions
    std::vector<Box> robot_links = {
        Box("link1", (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, angle1, robot_.getL1(), robot_.getW()),
        Box("link2", (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, angle2, robot_.getL2(), robot_.getW()),
        Box("link3", (p2[0] + p3[0]) / 2, (p2[1] + p3[1]) / 2, angle3, robot_.getL3(), robot_.getW())
    };


    // Check for collision with each surrounding box, excluding the target box
    for (const auto& link : robot_links) {
        for (const auto& box : environment_.getBoxes()) {
            if (box.getUID() == target_box_uid) {
                continue;
            }
            if (areBoxesColliding(link, box, clearance_threshold_)) {
                return true;
            }
        }
    }

    return false;
}

bool CollisionChecker::isTargetBoxInCollision(const Box& target_box) const {
    // Check for collision with each surrounding box
    for (const auto& box : environment_.getBoxes()) {
        //skip self collision
        if (box.getUID() == target_box.getUID()) {
            continue;
        }
        if (areBoxesColliding(target_box, box, clearance_threshold_)) {
            return true;
        }
    }

    return false;
}

Eigen::Vector3d CollisionChecker::getAttachedBoxPose(const Eigen::Vector3d& joint_angles, const Box& box) const {
    // Get the end-effector position from forward kinematics
    Eigen::Vector3d end_effector_pose = robot_.forwardKinematics(joint_angles);

    // Assume the box is attached such that its center is at the end-effector position
    return Eigen::Vector3d(end_effector_pose[0], end_effector_pose[1], end_effector_pose[2]);
}

void CollisionChecker::setClearanceThreshold(double clearance_threshold) {
    clearance_threshold_ = clearance_threshold;
}