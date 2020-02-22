cmake_minimum_required(VERSION 2.8.3)
project(srp_md)

# CXX
list(APPEND CMAKE_CXX_FLAGS
  -std=c++11
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
list(APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake_modules
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  srp_md_msgs
  std_msgs
  control_msgs
  dope_msgs
  geometry_msgs
  vision_msgs
  jsk_recognition_msgs
  tf
  tf_conversions
  tf2
  tf2_geometry_msgs
  tf2_eigen
  py_trees_ros
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  cv_bridge
)
find_package(Graphviz REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(PCL 1.8.1 REQUIRED)

## This macro ensures modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srp_md
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    srp_md_msgs
    std_msgs
    control_msgs
    dope_msgs
    geometry_msgs
    vision_msgs
    jsk_recognition_msgs
    tf
    tf_conversions
    tf2
    tf2_geometry_msgs
    tf2_eigen
    py_trees_ros
    moveit_core
    moveit_msgs
    moveit_ros_move_group
    moveit_ros_planning
    moveit_ros_planning_interface
    pcl_conversions
    pcl_ros
    cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########
# TODO(Kevin): Use a find_package or some other way to get these libraries
set(DAI_INCLUDE_DIRS /home/user/catkin_ws/src/mobile_manipulation/srp-md/submodules/libDAI/include)
# Note order is important
list(APPEND DAI_LIBRARIES
  /home/user/catkin_ws/src/mobile_manipulation/srp-md/submodules/libDAI/build/libscenegraph.a
  /home/user/catkin_ws/src/mobile_manipulation/srp-md/submodules/libDAI/lib/libdai.a
)
link_directories(/home/user/catkin_ws/src/mobile_manipulation/srp-md/submodules/libDAI/lib)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DAI_INCLUDE_DIRS}
  ${GRAPHVIZ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  /usr/include/glm/
  ${EIGEN3_INCLUDE_DIR}
)

# Factor Graph Node
add_executable(factor_graph_node
  src/factor_graph_node.cpp
  src/factor_graph_worker.cpp
)
# Note order should be if depends on another then must go higher in list
target_link_libraries(factor_graph_node
  ${catkin_LIBRARIES}
  ${GRAPHVIZ_CDT_LIBRARY}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${GRAPHVIZ_PATHPLAN_LIBRARY}
  ${OpenCV_LIBS}
  ${DAI_LIBRARIES}
  gmpxx
  gmp
)
add_dependencies(factor_graph_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Pose to scene graph node
add_executable(pose_to_scene_graph_node src/pose_to_scene_graph_node.cpp src/pose_to_scene_graph.cpp)
# Note order should be if depends on another then must go higher in list
target_link_libraries(pose_to_scene_graph_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(pose_to_scene_graph_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Pose to scene graph node
add_executable(plan_task src/plan_task.cpp)
# Note order should be if depends on another then must go higher in list
target_link_libraries(plan_task
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(plan_task
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Action node
add_executable(action_node
  src/action_node.cpp
  src/actions.cpp
)
# Note order should be if depends on another then must go higher in list
target_link_libraries(action_node
  ${catkin_LIBRARIES}
)
add_dependencies(action_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/srp_md_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
