#ifndef PLANAR_ARM_BOX_MOVING_BOX_HPP
#define PLANAR_ARM_BOX_MOVING_BOX_HPP

#include <Eigen/Dense>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <array>

/**
 * @class Box
 * @brief A class to represent a box in the environment.
 */
class Box {
public:
    /**
     * @brief Constructor to initialize the box with given properties.
     * @param uid Unique identifier for the box.
     * @param x X-coordinate of the box center.
     * @param y Y-coordinate of the box center.
     * @param alpha Orientation angle of the box (in radians).
     * @param w Width of the box.
     * @param h Height of the box.
     */
    Box(const std::string& uid, double x, double y, double alpha, double w, double h);

    /**
     * @brief Visualize the box using VTK.
     * @param color Color of the box (RGB array).
     * @return A vtkSmartPointer to the actor representing the box.
     */
    vtkSmartPointer<vtkActor> visualize(const std::array<double, 3>& color) const;

    /**
     * @brief Getter for the UID of the box.
     * @return UID of the box.
     */
    std::string getUID() const;

    /**
     * @brief Getter for the position of the box.
     * @return Position of the box as an Eigen::Vector3d (x, y, alpha).
     */
    Eigen::Vector3d getPosition() const;

    /**
     * @brief Getter for the size of the box.
     * @return Size of the box as an Eigen::Vector2d (w, h).
     */
    Eigen::Vector2d getSize() const;



private:
    std::string uid_;         ///< Unique identifier for the box.
    Eigen::Vector3d position_;
    Eigen::Vector2d size_;
};

#endif // PLANAR_ARM_BOX_MOVING_BOX_HPP