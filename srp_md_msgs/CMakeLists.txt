cmake_minimum_required(VERSION 2.8.3)
project(srp_md_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	message_generation
	actionlib_msgs
	geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
# Generate actions in the 'action' folder
add_action_files(
	DIRECTORY
		action
  FILES
  	DetectPlane.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  	actionlib_msgs
  	geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
	CATKIN_DEPENDS
		actionlib_msgs
		geometry_msgs
)
