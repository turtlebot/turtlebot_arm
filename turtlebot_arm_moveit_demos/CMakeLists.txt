cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_arm_moveit_demos)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS bin/pick_and_place.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
