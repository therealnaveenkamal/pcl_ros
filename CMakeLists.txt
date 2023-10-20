cmake_minimum_required(VERSION 3.0.2)
project(pcl_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(magic_subscriber_pcl2image_library
  src/magic_subscriber_pcl2image.cpp
)

add_executable(main_pcl2image_node src/main_pcl2image.cpp)


target_link_libraries(main_pcl2image_node
	magic_subscriber_pcl2image_library
	${catkin_LIBRARIES}
)