cmake_minimum_required(VERSION 3.1.3)
project(rasm_moveit3)

find_package(catkin REQUIRED industrial_trajectory_filters)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
