target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
)

add_executable(power_system_node power_system_node.cpp)

target_link_libraries(power_system_node
  ${catkin_LIBRARIES}
)

install(TARGETS power_system_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
