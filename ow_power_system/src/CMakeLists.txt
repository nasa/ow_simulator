cmake_minimum_required(VERSION 2.8.3)
project(ow_power_system)

add_definitions(-DUSING_ROS)

add_executable(power_system_node power_system_node.cpp)

target_link_libraries(power_system_node
	${catkin_LIBRARIES}
)

install(TARGETS power_system_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
