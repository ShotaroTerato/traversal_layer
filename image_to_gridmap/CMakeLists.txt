cmake_minimum_required(VERSION 2.8.3)
project(image_to_gridmap)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  filters
  nav_msgs
  roscpp
  sensor_msgs
  tf
  cmake_modules
  grid_map_ros
  grid_map_cv
  grid_map_msgs
)
find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(image_publisher src/image_publisher.cpp)
add_executable(image_to_gridmap src/image_to_gridmap.cpp src/ImageToGridmap.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(image_publisher
 ${catkin_LIBRARIES}
)
target_link_libraries(image_to_gridmap
 ${catkin_LIBRARIES}
)
