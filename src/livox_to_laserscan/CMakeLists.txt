cmake_minimum_required(VERSION 3.0.2)
project(livox_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp sensor_msgs geometry_msgs tf pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)
