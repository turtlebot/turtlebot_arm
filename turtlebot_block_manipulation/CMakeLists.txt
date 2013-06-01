cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_block_manipulation)


#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()


# setup
find_package(catkin REQUIRED roscpp cv_bridge image_transport tf pcl_ros)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS pcl_ros pcl roscpp)

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )

# node
#add_executable(chess_perception src/piece_finder.cpp
#                                src/board_finder.cpp
#                                src/chess_perception.cpp)
#target_link_libraries(chess_perception ${Boost_LIBRARIES}
#                                       ${catkin_LIBRARIES}
#                                       ${PCL_LIBRARIES}
#                                       ${OpenCV_LIBRARIES})
#add_dependencies(chess_perception chess_msgs_gencpp)


#rosbuild_genmsg()

#rosbuild_add_executable(block_manipulation test/block_manipulation.cpp)
##rosbuild_add_executable(block_detection_action_server src/block_detection_action_server.cpp)
#rosbuild_add_executable(interactive_manipulation_action_server src/interactive_manipulation_action_server.cpp)
#rosbuild_add_executable(pick_and_place_action_server src/pick_and_place_action_server.cpp)
#rosbuild_add_executable(block_manipulation_actions test/block_manipulation_actions.cpp)
#rosbuild_add_executable(block_manipulation_action_demo demo/block_manipulation_action_demo.cpp)

