cmake_minimum_required(VERSION 3.0.2)
project(rosbag_sync_reader)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 LIBRARIES coordinate Opencv
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs cv_bridge image_transport
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rosbag_sync_reader src/rosbag_sync_reader.cpp)
add_dependencies(rosbag_sync_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_sync_reader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
)