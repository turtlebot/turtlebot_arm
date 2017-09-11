cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_arm_ikfast_plugin)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  )
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  LIBRARIES
  DEPENDS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS})

set(MOVEIT_LIB_NAME turtlebot_arm_moveit_ikfast_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/turtlebot_arm_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(FILES turtlebot_arm_moveit_ikfast_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)















