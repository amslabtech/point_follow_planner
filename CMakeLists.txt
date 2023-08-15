cmake_minimum_required(VERSION 3.0.2)
project(point_follow_planner)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  jsk_recognition_msgs
  tf
)

find_package(Eigen3 REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES point_follow_planner_lib
    # CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
    # DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
add_library(point_follow_planner_lib src/point_follow_planner.cpp)

## Add cmake target dependencies of the library
add_dependencies(point_follow_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(point_follow_planner  src/point_follow_planner_node.cpp)
add_executable(robot_footprint_publisher src/robot_footprint_publisher_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(point_follow_planner  ${roscpp_LIBRARIES} ${catkin_LIBRARIES} point_follow_planner_lib)
target_link_libraries(robot_footprint_publisher ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
