cmake_minimum_required(VERSION 2.8.3)
project(grasp_gen)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pluginlib
  geometry_msgs
  moveit_planner
  moveit_ros_planning_interface
  moveit_msgs
  moveit_ros_planning
  moveit_ros_move_group
  tf
  franka_gripper
  )

  find_package(PCL REQUIRED)# COMPONENTS
  #common
  #io
#)

find_package(Boost REQUIRED)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )
catkin_package(DEPENDS moveit_planner)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(control_node src/controller.cpp)
add_executable(setup_node src/setup.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)



add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(setup_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_link_libraries(control_node ${catkin_LIBRARIES})
target_link_libraries(setup_node ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
