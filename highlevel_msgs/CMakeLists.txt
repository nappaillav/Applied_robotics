cmake_minimum_required(VERSION 3.0.2)
project(highlevel_msgs)


find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  actionlib_msgs
)


## Generate services in the 'srv' folder
add_service_files(
	FILES
	MoveTo.srv
)

add_action_files(
	DIRECTORY 
		action
	FILES 
		PoseCommand.action		
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs'
   actionlib_msgs
)

#
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES highlevel_msgs
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

