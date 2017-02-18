cmake_minimum_required(VERSION 2.8.3)
project(icub_moveit)

#find_package(catkin REQUIRED)

#catkin_package()

find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
	     yarp_msg
	    
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    yarp_msg
    
)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(motion_planning src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
