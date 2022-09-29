cmake_minimum_required(VERSION 3.0.2)
project(wt901b_ros)

#############
## Package ##
#############

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation #この行を追加
)

add_message_files(
  FILES
  wt901b.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime)



###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/wt901b_ros.cpp src/serial_communication.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
