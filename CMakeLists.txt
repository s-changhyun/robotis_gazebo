cmake_minimum_required(VERSION 2.8.3)
project(robotis_gazebo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    gazebo_ros
    urdf
    roscpp
    dynamixel_sdk
    robotis_framework_common
    robotis_device
    robotis_controller
    robotis_controller_msgs
    robotis_math
    cmake_modules
    robotis_op3_kinematics_dynamics
    #scilab_optimization
    robotis_op3_online_walking
    robotis_op3_motion_module
)

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    urdf
  DEPENDS
    gazebo
  LIBRARIES
    robotis_gazebo_plugin
  INCLUDE_DIRS
    include
)

add_library(robotis_gazebo_plugin src/plugin.cpp)
target_link_libraries(robotis_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS robotis_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY config include launch robots worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
