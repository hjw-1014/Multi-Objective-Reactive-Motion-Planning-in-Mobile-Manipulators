cmake_minimum_required(VERSION 3.0.2)
project(iiwas_bringup)


find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES env.sh
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


