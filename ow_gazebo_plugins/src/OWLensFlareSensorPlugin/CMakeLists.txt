set(TARGET_NAME OWLensFlareSensorPlugin)

set (HEADERS
  OWLensFlareSensorPlugin.h
)

set (SOURCES
  OWLensFlareSensorPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_PLUGIN_PATH}/libLensFlareSensorPlugin.so
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
