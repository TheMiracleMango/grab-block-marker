cmake_minimum_required(VERSION 2.8.3)
project(grab_block_marker)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  gazebo_ros
  roscpp
  tf2
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  holding.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS
   interactive_markers
   gazebo_ros
   roscpp tf2
   message_runtime
   geometry_msgs
   std_msgs
   visualization_msgs
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/grab_block_marker.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/grab_block_marker_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)
