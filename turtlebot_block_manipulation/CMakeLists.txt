cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_block_manipulation)

# setup
find_package(catkin REQUIRED actionlib actionlib_msgs interactive_markers pcl pcl_ros roscpp visualization_msgs)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )

add_action_files(FILES
                 BlockDetection.action
                 InteractiveBlockManipulation.action
                 PickAndPlace.action
                 )

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)
catkin_package(DEPENDS actionlib actionlib_msgs interactive_markers pcl pcl_ros roscpp visualization_msgs)


# node
add_executable(block_detection_action_server src/block_detection_action_server.cpp)
target_link_libraries(block_detection_action_server ${catkin_LIBRARIES}
                                                    ${PCL_LIBRARIES})
add_dependencies(block_detection_action_server turtlebot_block_manipulation_gencpp)
