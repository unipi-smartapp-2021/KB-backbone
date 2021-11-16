# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.7...3.21)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})  
endif()

project(backbone)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation topic_tools)


add_service_files(
  FILES
  ThrottleTopic.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(echo_server src/echo_server.cpp)
target_link_libraries(echo_server ${catkin_LIBRARIES})

add_executable(echo_client src/echo_client.cpp)
target_link_libraries(echo_client ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
