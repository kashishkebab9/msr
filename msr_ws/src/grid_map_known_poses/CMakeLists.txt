cmake_minimum_required(VERSION 3.0.2)
project(grid_map_known_poses)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  laser_geometry
  visualization_msgs
  grid_map_ros
  grid_map_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs


)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  grid_map_msgs
  grid_map_ros
  roscpp
  sensor_msgs
  visualization_msgs
  std_msgs
  laser_geometry
  tf2_ros
  tf2
  tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan_pcl src/scan_to_pcl.cpp)
target_link_libraries(scan_pcl ${catkin_LIBRARIES})

add_executable(gmwkp src/gmwkp.cpp)
target_link_libraries(gmwkp ${catkin_LIBRARIES})

install(TARGETS
  gmwkp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

