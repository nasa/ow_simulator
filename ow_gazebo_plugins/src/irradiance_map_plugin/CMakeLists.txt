set (HEADERS
  IrradianceMapPlugin.h
)

set (SOURCES
  IrradianceMapPlugin.cpp
  CubemapFilter.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(irradiance_map_plugin SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(irradiance_map_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
set_target_properties(irradiance_map_plugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
