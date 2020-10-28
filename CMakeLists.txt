cmake_minimum_required(VERSION 2.8.3)
project(uav_frontier_exploration_3d)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
  geographic_msgs
  octomap_msgs
  octomap_ros
  nodelet
  mean_shift_clustering
)
find_package(Boost)
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    sample_consensus
    io
    ros
    segmentation
    filters)
find_package(octomap 1.7 REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(cmake_modules REQUIRED)

generate_messages(
    DEPENDENCIES
    std_msgs
    geographic_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES frontier_exploration_3d
    CATKIN_DEPENDS octomap_ros pcl_ros
    DEPENDS octomap PCL)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
 )

link_directories(
    ${PCL_LIBRARY_DIRS})
add_definitions(
    ${PCL_DEFINITIONS})  

set(LINK_LIBS
    ${OCTOMAP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_library(frontier_exploration_3d 
    src/FrontierServer.cpp)
target_link_libraries(frontier_exploration_3d ${LINK_LIBS} yaml-cpp)

add_executable(frontier_server_node src/FrontierServerNode.cpp src/OctomapServer.cpp src/BestFrontier.cpp)
target_link_libraries(frontier_server_node frontier_exploration_3d ${LINK_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
)
      