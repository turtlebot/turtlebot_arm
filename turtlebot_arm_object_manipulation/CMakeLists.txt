cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_arm_object_manipulation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# setup
find_package(catkin REQUIRED roscpp actionlib actionlib_msgs interactive_markers object_recognition_msgs visualization_msgs
                             geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface yocs_math_toolkit geometric_shapes shape_msgs)
find_package(Boost REQUIRED system filesystem)
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                   )

add_action_files(FILES
                 InteractiveManip.action
                 ObjectDetection.action
                 PickAndPlace.action
                 MoveToTarget.action
                 UserCommand.action
                 )

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)
catkin_package(DEPENDS roscpp actionlib actionlib_msgs interactive_markers object_recognition_msgs visualization_msgs arbotix_msgs
                       geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface yocs_math_toolkit geometric_shapes shape_msgs)


# nodes
add_executable(object_detection_action_server src/object_detection_action_server.cpp)
target_link_libraries(object_detection_action_server ${catkin_LIBRARIES})
add_dependencies(object_detection_action_server turtlebot_arm_object_manipulation_gencpp)

add_executable(object_interactive_manip_action_server src/interactive_manipulation_action_server.cpp)
target_link_libraries(object_interactive_manip_action_server ${catkin_LIBRARIES})
add_dependencies(object_interactive_manip_action_server turtlebot_arm_object_manipulation_gencpp)

add_executable(object_pick_and_place_action_server src/pick_and_place_action_server.cpp)
target_link_libraries(object_pick_and_place_action_server ${catkin_LIBRARIES})
add_dependencies(object_pick_and_place_action_server turtlebot_arm_object_manipulation_gencpp)

# test  TODO :  needs review; probably not working
add_executable(object_manipulation_test_actions test/object_manipulation_actions.cpp)
target_link_libraries(object_manipulation_test_actions ${catkin_LIBRARIES})
add_dependencies(object_manipulation_test_actions turtlebot_arm_object_manipulation_gencpp)


# install

install(TARGETS object_detection_action_server object_interactive_manip_action_server object_pick_and_place_action_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS python/keyboard_user_commands.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY demo
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
