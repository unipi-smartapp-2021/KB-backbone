cmake_minimum_required(VERSION 3.7...3.21)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})  
endif()

project(backbone)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

catkin_python_setup()
add_service_files(
  DIRECTORY srv
  FILES
  RateTopic.srv
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

catkin_install_python(
  PROGRAMS scripts/client.py scripts/server.py scripts/source.py scripts/debugger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(Source src/source.cpp)
target_link_libraries(Source ${catkin_LIBRARIES})

add_executable(Multiplexer src/multiplexer.cpp)
target_link_libraries(Multiplexer ${catkin_LIBRARIES})

add_executable(FirstClient src/first_client.cpp)
target_link_libraries(FirstClient ${catkin_LIBRARIES})

add_executable(SecondClient src/second_client.cpp)
target_link_libraries(SecondClient ${catkin_LIBRARIES})

add_executable(Debugger src/debugger.cpp)
target_link_libraries(Debugger ${catkin_LIBRARIES})
