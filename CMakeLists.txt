cmake_minimum_required(VERSION 3.0.2)
project(point_follow_planner)

add_compile_options(-std=c++17 -O2 -g)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
)

find_package(Eigen3 REQUIRED COMPONENTS system)


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

## Specify libraries to link a library or executable target against
target_link_libraries(point_follow_planner  ${roscpp_LIBRARIES} ${catkin_LIBRARIES} point_follow_planner_lib)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
endif()
