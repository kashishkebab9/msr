cmake_minimum_required(VERSION 3.0.2)
project(grid_map_known_poses)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
)

catkin_package()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(gmwkp src/gmwkp.cpp)
target_link_libraries(gmwkp ${catkin_LIBRARIES})

