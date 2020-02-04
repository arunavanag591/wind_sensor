cmake_minimum_required(VERSION 2.8.3)
project(limit_switch)


## Compile as C++11, supported in ROS Kinetic and newer
##set(CMAKE_CXX_STANDARD 11)
## list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -fprofile-arcs -ftest-coverage")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  roslib
  roscpp
  phidgets_api
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES phidgets_api
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs roslib     
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -fprofile-arcs -ftest-coverage")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  roslib
  roscpp
  phidgets_api
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES phidgets_api
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs roslib     
)

include_directories(include ${catkin_INCLUDE_DIRS})

## fixture library ##
add_library(${PROJECT_NAME} src/motor_fixture.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## fixture node ##
add_executable(${PROJECT_NAME}_node src/motor_fixture_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME} )
target_link_libraries(${PROJECT_NAME}_node ${GTEST_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
##

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

 )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
endif()
