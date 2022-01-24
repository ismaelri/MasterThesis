cmake_minimum_required(VERSION 2.8.3)
project(my_files)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    rospy
    std_msgs
    message_generation
    my_ur10_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)


add_message_files(
   FILES
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES
    interactivity_utils
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3 my_ur10_msgs
)

###########
## Build ##
###########

include_directories(SYSTEM include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(ros_PBC src/ros_PBC.cpp)
target_link_libraries(ros_PBC ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_PBC DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ros_PBC my_files_generate_messages_cpp)

add_executable(ros_PBVC src/ros_PBVC.cpp)
target_link_libraries(ros_PBVC ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_PBVC DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ros_PBVC my_files_generate_messages_cpp)

add_executable(ros_IBVC src/ros_IBVC.cpp)
target_link_libraries(ros_IBVC ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_IBVC DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ros_IBVC my_files_generate_messages_cpp)

add_executable(ros_IBVC_CC src/ros_IBVC_CC.cpp)
target_link_libraries(ros_IBVC_CC ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_IBVC_CC DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ros_IBVC_CC my_files_generate_messages_cpp)

add_executable(ros_HVC src/ros_HVC.cpp)
target_link_libraries(ros_HVC ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_HVC DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ros_HVC my_files_generate_messages_cpp)

add_executable(goTo src/goTo.cpp)
target_link_libraries(goTo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS goTo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(goTo my_files_generate_messages_cpp)


catkin_install_python(PROGRAMS scripts/plots.py scripts/save_desired_position.py scripts/send_desired_position.py scripts/save_desired_image.py scripts/send_desired_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
