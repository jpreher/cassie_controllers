cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cassie_controllers)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# Custom commands for specifying relative link_directories
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
  cassie_description
  control_utilities
  eigen_utilities
  ros_utilities
  cassie_common_toolbox
  cassie_estimation
  realtime_utilities
  roslib_utilities
  qpoases_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy std_srvs ros_utilities eigen_utilities control_utilities cassie_common_toolbox realtime_utilities roslib_utilities qpoases_ros
   DEPENDS
)

###########
## Build ##
###########
add_definitions(-Wno-deprecated-declarations)

include(common_flags)
option(USE_VECTORIZATION "Use vectorization" OFF)
if(USE_VECTORIZATION)
  common_add_vectorization_flags()
endif()

# Specify custom includes and libs
include_directories(include ${catkin_INCLUDE_DIRS})

# Create controller libraries
add_library(${PROJECT_NAME}
    src/standing_control_qp.cpp
    src/walking_onedomain.cpp
    src/inverse_kinematics.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Main state machine node for handling locomotion controllers
add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node ${catkin_LIBRARIES} ${PROJECT_NAME})
