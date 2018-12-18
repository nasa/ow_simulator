find_package(roscpp REQUIRED)

set (HEADERS
  NearFarClipPlugin.h
)

set (SOURCES
  NearFarClipPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(near_far_clip_plugin SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(near_far_clip_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} Qt5::Widgets)
set_target_properties(near_far_clip_plugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
