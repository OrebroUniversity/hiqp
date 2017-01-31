cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(hiqp_client)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  hiqp_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs hiqp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Main library
add_library(hiqp_client src/hiqp_client.cpp)
add_dependencies(hiqp_client hiqp_msgs_generate_messages_cpp)
target_link_libraries(hiqp_client ${catkin_LIBRARIES})

# Test frame on frame.
add_executable (test_frameonframe src/test_frameonframe.cpp)
add_dependencies(test_frameonframe hiqp_msgs_generate_messages_cpp)
target_link_libraries(test_frameonframe hiqp_client ${catkin_LIBRARIES})
