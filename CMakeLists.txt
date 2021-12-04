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
  PROGRAMS scripts/multiplexer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(Source test_nodes/cpp/source.cpp)
add_dependencies(Source ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Source ${catkin_LIBRARIES})

add_executable(Multiplexer test_nodes/cpp/multiplexer.cpp)
add_dependencies(Multiplexer ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Multiplexer ${catkin_LIBRARIES})

add_executable(FirstClient test_nodes/cpp/first_client.cpp)
add_dependencies(FirstClient ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(FirstClient ${catkin_LIBRARIES})

add_executable(SecondClient test_nodes/cpp/second_client.cpp)
add_dependencies(SecondClient ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(SecondClient ${catkin_LIBRARIES})

add_executable(Debugger test_nodes/cpp/debugger.cpp)
target_link_libraries(Debugger ${catkin_LIBRARIES})
