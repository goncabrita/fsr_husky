cmake_minimum_required(VERSION 2.8.3)
project(odo_covariance)

find_package(catkin REQUIRED roscpp sensor_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_offset
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(odo_covariance src/odo_covariance.cpp)

target_link_libraries(odo_covariance
   ${catkin_LIBRARIES}
)

