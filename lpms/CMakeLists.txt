cmake_minimum_required(VERSION 2.8.3)
project(lpms)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lpms
 CATKIN_DEPENDS roscpp std_msgs controller
#  DEPENDS system_lib
  
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
#add_executable(lpms_node lpms_node src/lpms.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(lpms_node
#  ${catkin_LIBRARIES}
#  LpSensor.so
#  pthread
#  dl
#  rt
#)


## Declare a C++ executable
add_executable(lpms_node lpms_node src/lpms_v1.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lpms_node
  ${catkin_LIBRARIES}
  LpSensor.so
  pthread
  dl
  rt
)

