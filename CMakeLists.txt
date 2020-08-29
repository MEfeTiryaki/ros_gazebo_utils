cmake_minimum_required(VERSION 2.8.3)
project(ros_gazebo_utils)

add_definitions("-std=c++11")

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED
COMPONENTS
  roscpp
  gazebo_plugins
  gazebo_ros
  urdf
  xacro
  tf
  ros_node_utils
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES # only header files, therefore no libraries
  CATKIN_DEPENDS
    roscpp
    gazebo_plugins
    gazebo_ros
    urdf
    xacro
    tf
    ros_node_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  include
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)
