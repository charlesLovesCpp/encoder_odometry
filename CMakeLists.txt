cmake_minimum_required(VERSION 2.8.3)
project(encoder_odometry)
set (CMAKE_CXX_STANDARD 14)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(encoder_odometry_node 
  src/encoder_odometry_node.cpp)
target_link_libraries(encoder_odometry_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
)

add_executable(encoder_publisher_node 
  src/encoder_publisher_node.cpp)
target_link_libraries(encoder_publisher_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
)

add_executable(amcl_reader_node 
  src/amcl_reader_node.cpp)
target_link_libraries(amcl_reader_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
)

add_executable(cartographer_subscriber_node 
  src/cartographer_subscriber_node.cpp)
target_link_libraries(cartographer_subscriber_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
)