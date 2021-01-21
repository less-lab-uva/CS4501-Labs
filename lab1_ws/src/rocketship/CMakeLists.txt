cmake_minimum_required(VERSION 2.8.3)
project(rocketship)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -o3 -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs
)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(countdown src/countdown.cpp)
target_link_libraries(countdown ${catkin_LIBRARIES})
add_dependencies(countdown rocketship_generate_messages_cpp)

add_executable(main_controller src/main_controller.cpp)
target_link_libraries(main_controller ${catkin_LIBRARIES})
add_dependencies(main_controller rocketship_generate_messages_cpp)