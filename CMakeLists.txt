cmake_minimum_required(VERSION 3.0.2)
project(uml_hri_nerve_pick_and_place)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  message_generation
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  gpd_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#add_message_files(
#  FILES
#)

add_service_files(
  FILES
  CapturePointCloud.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

add_executable(pick_and_place
	src/nodes/pick_and_place.cpp
  src/functions/navigation_class.cpp
	src/functions/manipulation_class.cpp
        src/functions/perception_class.cpp
        src/functions/pointcloud_filters.cpp
        src/functions/grasp_cluster_class.cpp)
add_executable(gen3_pick_and_place
        src/nodes/gen3_pick_and_place.cpp
        src/functions/navigation_class.cpp
        src/functions/manipulation_class.cpp
        src/functions/perception_class.cpp
        src/functions/pointcloud_filters.cpp
        src/functions/grasp_cluster_class.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES})
target_link_libraries(gen3_pick_and_place ${catkin_LIBRARIES})

add_executable(capture_pointcloud_service src/services/capture_pointcloud_service.cpp)
target_link_libraries(capture_pointcloud_service ${catkin_LIBRARIES})
add_dependencies(capture_pointcloud_service uml_hri_nerve_pick_and_place_gencpp)

#add_executable(add_two_ints_client src/nodes/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client uml_hri_nerve_pick_and_place_gencpp)
