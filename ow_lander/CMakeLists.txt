cmake_minimum_required(VERSION 2.8.3)
project(ow_lander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

find_package(roslaunch)

add_service_files(
  FILES
  DeliverSample.srv
  DigCircular.srv
  DigLinear.srv
  Grind.srv
  GuardedMove.srv
  PublishTrajectory.srv
  Stow.srv
  Unstow.srv
)

add_message_files(
  FILES
  GuardedMoveResult.msg
  # SystemFaults.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

catkin_add_env_hooks(
  65.ow_lander
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir config meshes urdf)
        install(DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

# add_dependencies(ow_faults ${catkin_EXPORTED_TARGETS})
# add_dependencies (
#   ${EXEC_NAME} ${PROJECT_NAME}_gencfg
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   )