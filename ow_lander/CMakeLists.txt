cmake_minimum_required(VERSION 2.8.3)

project(ow_lander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(roslaunch)

add_service_files(
  FILES
  StartPlanning.srv
  PublishTrajectory.srv
  MoveGuarded.srv
  MoveReset.srv
  DigTrench.srv
  SawMotionPlanning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

catkin_add_env_hooks(
  65.ow_lander
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

foreach(dir config launch meshes urdf)
        install(DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
