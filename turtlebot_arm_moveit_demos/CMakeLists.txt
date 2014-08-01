cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_arm_moveit_demos)

find_package(catkin REQUIRED COMPONENTS
	tf
	rospy
	geometry_msgs
	trajectory_msgs
	moveit_msgs
	moveit_commander
)

catkin_package()

install(PROGRAMS bin/pick_and_place.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})