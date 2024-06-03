#ifndef PLANAR_ARM_BOX_MOVING_ENVIRONMENT_HPP
#define PLANAR_ARM_BOX_MOVING_ENVIRONMENT_HPP

#include "box.h"
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <array>

/**
 * @class Environment
 * @brief A class to represent the environment containing multiple boxes.
 */
class Environment {
public:
    /**
     * @brief Constructor to initialize the environment with a default box color.
     * @param default_color The default color for the boxes (RGB array).
     */
    Environment(const std::array<double, 3>& default_color);

    /**
     * @brief Add a box to the environment.
     * @param box The box to add.
     */
    void addBox(const Box& box);

    /**
     * @brief Visualize the environment using VTK.
     * @param renderer The VTK renderer to add the boxes to.
     */
    void visualize(vtkSmartPointer<vtkRenderer> renderer) const;

    /**
     * @brief Getter for the list of boxes in the environment.
     * @return The list of boxes in the environment.
     */
    const std::vector<Box>& getBoxes() const;

    const Box getBox(const std::string& uid) const;

private:
    std::vector<Box> boxes_; ///< List of boxes in the environment.
    std::array<double, 3> default_color_; ///< Default color for the boxes.
};

#endif // PLANAR_ARM_BOX_MOVING_ENVIRONMENT_HPP
