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

find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

add_service_files(
  FILES
  DeliverSample.srv
  DigCircular.srv
  DigLinear.srv
  Grind.srv
  GuardedMove.srv
  Light.srv
  PublishTrajectory.srv
  Stop.srv
  Stow.srv
  Unstow.srv
)

add_action_files(
  DIRECTORY action
  FILES Unstow.action
  FILES Stow.action
  FILES Deliver.action
  FILES DigLinear.action
  FILES DigCircular.action
  FILES Grind.action
  FILES GuardedMove.action
  FILES AntennaPanTilt.action
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
  CATKIN_DEPENDS actionlib_msgs
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
