cmake_minimum_required(VERSION 2.8.3)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
#   Message1.msg
#   Message2.msg
  FloatList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Setpoints.cfg
  cfg/Controller.cfg
  cfg/Mixer.cfg
)


###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(setpoints src/setpoints.cpp)
target_link_libraries(setpoints ${catkin_LIBRARIES})
add_dependencies(setpoints ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(attitude src/attitude.cpp)
target_link_libraries(attitude ${catkin_LIBRARIES})
add_dependencies(attitude ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(position src/position.cpp)
target_link_libraries(position ${catkin_LIBRARIES})
add_dependencies(position ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mixer src/mixer.cpp)
target_link_libraries(mixer ${catkin_LIBRARIES})
add_dependencies(mixer ${${PROJECT_NAME}_EXPORTED_TARGETS})


