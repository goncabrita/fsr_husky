cmake_minimum_required(VERSION 2.8.3)
project(fsr_husky_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS laser_assembler
                                        roscpp
                                        angles
                                        cereal_port
                                        sensor_msgs
                                        actionlib
                                        actionlib_msgs
                                        control_msgs
                                        moveit_core
                                        moveit_ros_planning_interface
                                        moveit_ros_perception
                                        hardware_interface
                                        controller_manager
                                        pcl_ros
                                        nanotec_driver
                                        dynamixel_msgs
                                        message_generation
                                        std_msgs
                                        rospy)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(
   DIRECTORY action
   FILES Home.action
   Goto.action
   laser_tilt.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   sensor_msgs
   dynamixel_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fsr_husky_driver
#  CATKIN_DEPENDS laser_assembler roscpp
  DEPENDS actionlib_msgs actionlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cereal_port_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(fsr_husky_driver
#   src/${PROJECT_NAME}/fsr_husky_driver.cpp
# )

## Declare a cpp executable
add_executable(cloud_assembler_node src/pointcloud_generator/cloud_assembler.cpp)
#add_executable(cloud_assembler_node_v0 src/pointcloud_generator/cloud_assembler_v0.cpp)
add_executable(laser_tilt_action_server_node src/pointcloud_generator/laser_tilt_action_server.cpp)
add_executable(tilt_continuous_operation_client_node src/pointcloud_generator/tilt_continuous_operation_client.cpp)

add_executable(fsr_husky_arm_node src/mine_clearance_arm/fsr_husky_arm_node.cc src/mine_clearance_arm/jrk_driver.cc)
add_executable(home_node src/mine_clearance_arm/home_node.cpp)
add_executable(sweep_arm_node src/mine_clearance_arm/sweep_node.cpp)
add_executable(sweep_arm_node_v0 src/mine_clearance_arm/sweep_node_v0.cpp)
add_executable(sweep_arm_node_v2 src/mine_clearance_arm/sweep_node_v2.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
target_link_libraries(cloud_assembler_node ${catkin_LIBRARIES})
#target_link_libraries(cloud_assembler_node_v0 ${catkin_LIBRARIES})
target_link_libraries(fsr_husky_arm_node ${catkin_LIBRARIES} cereal_port nanotec_driver)
target_link_libraries(home_node ${catkin_LIBRARIES})
target_link_libraries(sweep_arm_node ${catkin_LIBRARIES})
target_link_libraries(sweep_arm_node_v0 ${catkin_LIBRARIES})
target_link_libraries(sweep_arm_node_v2 ${catkin_LIBRARIES})
target_link_libraries(laser_tilt_action_server_node ${catkin_LIBRARIES})
target_link_libraries(tilt_continuous_operation_client_node ${catkin_LIBRARIES})

add_dependencies(cloud_assembler_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(cloud_assembler_node_v0 ${catkin_EXPORTED_TARGETS})
add_dependencies(fsr_husky_arm_node ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_tilt_action_server_node ${fsr_husky_driver_EXPORTED_TARGETS})
add_dependencies(tilt_continuous_operation_client_node ${fsr_husky_driver_EXPORTED_TARGETS})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS fsr_husky_driver fsr_husky_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fsr_husky_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
