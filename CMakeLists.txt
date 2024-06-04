cmake_minimum_required(VERSION 3.10)

project(planar_robot_box_moving)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(VTK REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
include(${VTK_USE_FILE})

# Add the library
add_library(${PROJECT_NAME}
        src/planner.cpp
        src/utils.cpp
        src/collision_checker.cpp
        src/box.cpp
        src/environment.cpp
        src/robot_arm.cpp
)

# Add the executable
add_executable(test_planar_robot_box_moving
        test/test_robot_arm.cpp
        test/test_main.cpp)

add_executable(run_planner test/run_planner.cpp)

target_include_directories(run_planner PRIVATE include)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(test_planar_robot_box_moving PRIVATE include)

target_link_libraries(run_planner ${PROJECT_NAME} ${VTK_LIBRARIES} ${OMPL_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen )
target_link_libraries(test_planar_robot_box_moving ${PROJECT_NAME} Eigen3::Eigen GTest::GTest GTest::Main ${OMPL_LIBRARIES})