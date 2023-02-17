cmake_minimum_required(VERSION 3.0.2)
project(ow_lander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  tf2_ros
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES DockIngestSample.action
  FILES GuardedMove.action
  FILES Pan.action
  FILES Tilt.action
)

add_message_files(
  FILES
  GuardedMoveFinalResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs sensor_msgs
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
  PATTERN "setup_assistant.launch" EXCLUDE
)
