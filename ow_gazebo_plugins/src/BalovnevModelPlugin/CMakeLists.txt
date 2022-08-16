

set(TARGET_NAME BalovnevModelPlugin)

set (HEADERS
  BalovnevModelPlugin.h  
)

set (SOURCES
  BalovnevModelPlugin.cpp
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

