#include <planar_robot_box_moving/box.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>

Box::Box(const std::string& uid, double x, double y, double alpha, double w, double h)
    : uid_(uid), position_(x, y, alpha), size_(w, h) {}

std::string Box::getUID() const {
    return uid_;
}

Eigen::Vector3d Box::getPosition() const {
    return position_;
}

Eigen::Vector2d Box::getSize() const {
    return size_;
}


vtkSmartPointer<vtkActor> Box::visualize(const std::array<double, 3>& color) const {
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    cubeSource->SetXLength(size_[0]);
    cubeSource->SetYLength(size_[1]);
    cubeSource->SetZLength(0.1);

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(position_[0], position_[1], 0);
    transform->RotateZ(position_[2] * 180.0 / M_PI);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(cubeSource->GetOutputPort());
    transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetDiffuseColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetAmbientColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetSpecularColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetEdgeVisibility(1);

    return actor;
}