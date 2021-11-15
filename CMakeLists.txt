# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.7...3.21)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})  
endif()

project(backbone)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg topic_tools)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_executable(generic_pub src/generic_pub.cpp)
target_link_libraries(generic_pub ${catkin_LIBRARIES})
# %EndTag(FULLTEXT)%
