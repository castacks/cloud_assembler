cmake_minimum_required(VERSION 3.0.2)
project(cloud_assembler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  tf
  tf2
  tf2_ros
  tf_conversions
  interactive_markers
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shimizu_reloc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/shimizu_reloc.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(shimizu_reloc_node src/shimizu_reloc_node.cpp)
target_link_libraries(shimizu_reloc_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
add_executable(cloud_assembler_node src/cloud_assembler_node.cpp)
target_link_libraries(cloud_assembler_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
add_executable(main_assembler_node src/main_assembler_node.cpp)
target_link_libraries(main_assembler_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
add_executable(piece_locator_node src/piece_locator_node.cpp)
target_link_libraries(piece_locator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})