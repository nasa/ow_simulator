cmake_minimum_required(VERSION 2.8.3)
project(cosimulation_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(gazebo REQUIRED)
find_package(OGRE REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(TARGET_NAME CosimulationPlugin)

set (HEADERS
  Cosimulator.h
  CosimulationPlugin.h
)

set (SOURCES
  Cosimulator.cpp
  CosimulationPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

