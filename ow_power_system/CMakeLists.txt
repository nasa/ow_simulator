cmake_minimum_required(VERSION 2.8.3)
project(ow_power_system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
	roslib
)

list(INSERT CMAKE_MODULE_PATH 0
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp roslib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
