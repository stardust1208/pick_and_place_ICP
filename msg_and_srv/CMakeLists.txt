cmake_minimum_required(VERSION 3.0.2)
project(msg_and_srv)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  PointCloudArr.msg
  PointCloudArrSet.msg
  PointCloud2Set.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msg_and_srv
 CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
