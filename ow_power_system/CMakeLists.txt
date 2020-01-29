cmake_minimum_required(VERSION 2.8.3)
project(ow_power_system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(power_system power_system.cpp)

target_link_libraries(power_system 
   ${catkin_LIBRARIES}
)
