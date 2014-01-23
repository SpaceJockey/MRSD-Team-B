cmake_minimum_required(VERSION 2.8.3)
project(rqt_sj)

find_package(catkin REQUIRED message_generation rospy std_msgs)
catkin_python_setup()

add_message_files(
  FILES
  GaitComm.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)




install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_sj
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



