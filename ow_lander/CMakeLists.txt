cmake_minimum_required(VERSION 2.8.3)
 
project(ow_lander)
 
find_package(catkin REQUIRED)
find_package(roslaunch)

catkin_add_env_hooks(
  65.ow_lander
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

catkin_package()

foreach(dir config launch meshes urdf)
        install(DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
