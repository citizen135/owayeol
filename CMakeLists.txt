cmake_minimum_required(VERSION 3.0.2)
project(owayeol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rospy
  move_base_msgs
  geometry_msgs
  std_msgs
  message_generation
  rqt_gui
  rqt_gui_py
)

add_message_files(
  FILES
  YoloResult.msg
  ChangeRobot.msg
  RobotState.msg
  Trigger.msg
)

catkin_python_setup()

#catkin_package(
#  CATKIN_DEPENDS std_msgs message_runtime
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/owayeol
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
