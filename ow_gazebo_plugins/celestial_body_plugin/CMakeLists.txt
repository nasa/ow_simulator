cmake_minimum_required(VERSION 2.8.3)
project(celestial_body_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  tf2_ros
)

find_package(gazebo REQUIRED)
find_package(OGRE REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

set (HEADERS
  CelestialBodyPlugin.h
)

set (SOURCES
  CelestialBodyPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(celestial_body_plugin SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(celestial_body_plugin ${GAZEBO_libraries})
target_link_libraries(celestial_body_plugin ${catkin_LIBRARIES})
set_target_properties(celestial_body_plugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
