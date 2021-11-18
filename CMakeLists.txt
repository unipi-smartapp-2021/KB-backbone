cmake_minimum_required(VERSION 3.7...3.21)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})  
endif()

project(backbone)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation topic_tools)

add_service_files(
  FILES
  ThrottleTopic.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

# For `clangd`, change it with the correct path
# include_directories(/path_to_your_ros_ws/devel/include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(Source src/source.cpp)
target_link_libraries(Source ${catkin_LIBRARIES})

add_executable(Multiplexer src/multiplexer.cpp)
target_link_libraries(Multiplexer ${catkin_LIBRARIES})

add_executable(FirstClient src/first_client.cpp)
target_link_libraries(FirstClient ${catkin_LIBRARIES})

add_executable(SecondClient src/second_client.cpp)
target_link_libraries(SecondClient ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  #add_rostest(test/rostopic2.test)
  catkin_add_gtest(freq_multiplexer_test test/test_frequency_multiplexer.cpp)
endif()
