#include <planar_robot_box_moving/environment.h>


Environment::Environment(const std::array<double, 3>& default_color)
    : default_color_(default_color) {}

void Environment::addBox(const Box& box) {
    boxes_.push_back(box);
}

void Environment::visualize(vtkSmartPointer<vtkRenderer> renderer) const {
    for (const auto& box : boxes_) {
        renderer->AddActor(box.visualize(default_color_));
    }
}

const std::vector<Box>& Environment::getBoxes() const {
    return boxes_;
}

const Box Environment::getBox(const std::string& uid) const {
    for (const auto& box : boxes_) {
        if (box.getUID() == uid) {
            return box;
        }
    }
    throw std::invalid_argument("Box with UID " + uid + " not found in the environment.");
}