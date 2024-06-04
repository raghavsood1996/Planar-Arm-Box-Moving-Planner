#include <planar_robot_box_moving/utils.h>

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkProperty.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <vector>
#include <string>

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

vtkSmartPointer<vtkActor> createLink(double length, double width, const Eigen::Vector3d& start_position, double angle, const std::array<double, 3>& color) {
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    cubeSource->SetXLength(length);
    cubeSource->SetYLength(width);
    cubeSource->SetZLength(0.1); // Small thickness for 2D visualization

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(start_position[0], start_position[1], 0);
    transform->RotateZ(angle * 180.0 / M_PI); // Convert to degrees
    transform->Translate(length / 2.0, 0, 0); // Move to the center of the link

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(cubeSource->GetOutputPort());
    transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // Set the border color to black
    vtkSmartPointer<vtkProperty> property = actor->GetProperty();
    property->SetEdgeVisibility(1);
    property->SetEdgeColor(0, 0, 0); // Black color
    property->SetColor(color[0], color[1], color[2]);

    return actor;
}

void plotRobotArmAndEnvironment(const RobotArm& robot, const std::vector<Eigen::Vector3d>& path, const Environment& environment, const std::string& target_box_uid) {
    // Create a renderer
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    // Add environment to the renderer
    environment.visualize(renderer);

    for (size_t i = 0; i < path.size(); ++i) {
        if (i % 5 != 0 && i != path.size() -1) continue; // Skip every 5th point for rendering

        const auto& joint_angles = path[i];

        // Compute the positions and orientations of each link
        Eigen::Vector3d p0(0, 0, 0);

        double angle1 = joint_angles[0];
        Eigen::Vector3d p1 = p0 + Eigen::Vector3d(robot.getL1() * cos(angle1), robot.getL1() * sin(angle1), 0);

        double angle2 = angle1 + joint_angles[1];
        Eigen::Vector3d p2 = p1 + Eigen::Vector3d(robot.getL2() * cos(angle2), robot.getL2() * sin(angle2), 0);

        double angle3 = angle2 + joint_angles[2];
        Eigen::Vector3d p3 = p2 + Eigen::Vector3d(robot.getL3() * cos(angle3), robot.getL3() * sin(angle3), 0);

        auto link_color = std::array<double, 3>{0.1, 0.1, 0.1}; // Dark gray color for links
        auto box_color = std::array<double, 3>{0.1, 0.8, 0.1}; // Green color for boxes


        //set color to orange if its the last point
        if (i == path.size() - 1) {
            link_color = {0.8, 0.5, 0.1}; // Orange color for the last link
        }

        //det color to yellow if its the first point
        if (i == 0) {
            link_color = {0.8, 0.8, 0.1}; // Yellow color for the first link
            box_color = {0.1, 0.8, 0.8}; // Cyan color for box
        }

        // Create actors for each link
        vtkSmartPointer<vtkActor> link1 = createLink(robot.getL1(), robot.getW(), p0, angle1, link_color);
        vtkSmartPointer<vtkActor> link2 = createLink(robot.getL2(), robot.getW(), p1, angle2, link_color);
        vtkSmartPointer<vtkActor> link3 = createLink(robot.getL3(), robot.getW(), p2, angle3, link_color);

        renderer->AddActor(link1);
        renderer->AddActor(link2);
        renderer->AddActor(link3);


        //get dimensions of the target box
        Eigen::Vector2d target_box_size = environment.getBox(target_box_uid).getSize();

        // Create actor for the target box
        Box target_box = Box("attached_box", p3[0], p3[1], angle3, target_box_size[0], target_box_size[1]);
        

        renderer->AddActor(target_box.visualize(box_color));
    }

    

    renderer->SetBackground(0.1, 0.2, 0.4);  // Set background color

    // Create a render window
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Create a render window interactor
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Initialize and start the interactor
    renderWindow->Render();
    renderWindowInteractor->Start();
}

void loadEnvironment(const std::string& filename, Environment& environment) {
    YAML::Node env_config = YAML::LoadFile(filename);
    for (const auto& box : env_config["boxes"]) {
        environment.addBox(Box(
            box["uid"].as<std::string>(),
            box["x"].as<double>(),
            box["y"].as<double>(),
            box["alpha"].as<double>(),
            box["w"].as<double>(),
            box["h"].as<double>()
        ));
    }
}

void loadTestCases(const std::string& filename, std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::string, double>>& test_cases) {
    YAML::Node test_case_config = YAML::LoadFile(filename);
    for (const auto& test_case : test_case_config["test_cases"]) {
        Eigen::Vector3d start_pose(test_case["start_pose"][0].as<double>(), test_case["start_pose"][1].as<double>(), test_case["start_pose"][2].as<double>());
        Eigen::Vector3d goal_pose(test_case["goal_pose"][0].as<double>(), test_case["goal_pose"][1].as<double>(), test_case["goal_pose"][2].as<double>());
        std::string target_box_uid = test_case["target_box_uid"].as<std::string>();
        double clearance_threshold = test_case["clearance_threshold"].as<double>();
        test_cases.emplace_back(start_pose, goal_pose, target_box_uid, clearance_threshold);
    }
}