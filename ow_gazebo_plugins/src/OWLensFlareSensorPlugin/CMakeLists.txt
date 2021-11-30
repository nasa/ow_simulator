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

# Create path or path(s) to libLensFlareSensorPlugin.so
set(GAZEBO_PLUGIN_PATH $ENV{GAZEBO_PLUGIN_PATH})
if("${GAZEBO_PLUGIN_PATH}" STREQUAL "")
  message(FATAL_ERROR "GAZEBO_PLUGIN_PATH must be defined. You probably need to source /usr/share/gazebo/setup.bash")
endif()
string(REPLACE ":" ";" GAZEBO_PLUGIN_PATH_LIST ${GAZEBO_PLUGIN_PATH})
foreach(PATH ${GAZEBO_PLUGIN_PATH_LIST})
  list(APPEND LFSP_LIB "${PATH}/libLensFlareSensorPlugin.so")
endforeach()

target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${LFSP_LIB})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
