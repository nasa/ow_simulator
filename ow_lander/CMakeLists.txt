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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

foreach(dir config launch meshes urdf)
        install(DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)