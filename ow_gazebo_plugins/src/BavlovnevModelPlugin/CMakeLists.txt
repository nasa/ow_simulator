

set(TARGET_NAME BavlovnevModelPlugin)

set (HEADERS
  BavlovnevModelPlugin.h
  
)

set (SOURCES
  BavlovnevModelPlugin.cpp
)

include_directories(
  .
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)


target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

