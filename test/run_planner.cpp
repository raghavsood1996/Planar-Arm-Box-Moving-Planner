#include <planar_robot_box_moving/robot_arm.h>
#include <planar_robot_box_moving/environment.h>
#include <planar_robot_box_moving/collision_checker.h>
#include <planar_robot_box_moving/utils.h>
#include <planar_robot_box_moving/planner.h>
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


// Helper function to create a link as a cuboid
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


        //set color to orange if its the last point
        if (i == path.size() - 1) {
            link_color = {0.8, 0.5, 0.1}; // Orange color for the last link
        }

        //det color to yellow if its the first point
        if (i == 0) {
            link_color = {0.8, 0.8, 0.1}; // Yellow color for the first link
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
        

        renderer->AddActor(target_box.visualize({0.1, 0.8, 0.1}));
    }

    // Add environment to the renderer
    environment.visualize(renderer);

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