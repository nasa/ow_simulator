cmake_minimum_required(VERSION 3.0.2)

project(ow_bag_recorder)

find_package(catkin REQUIRED
  COMPONENTS roscpp rosbag
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rosbag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bag_recorder_node src/bag_recorder_node.cpp src/config.cpp)

add_dependencies(bag_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bag_recorder_node ${catkin_LIBRARIES})
