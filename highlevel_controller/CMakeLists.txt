cmake_minimum_required(VERSION 3.0.2)
project(highlevel_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  highlevel_msgs
  cubic_polynomial_planner
  actionlib 			# need this to execute ROS action
)


find_package(Eigen3 REQUIRED)						# find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")


catkin_package(
	INCLUDE_DIRS
    #	include  
    	${Eigen3_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
    	geometry_msgs
		actionlib   	 	    
  	DEPENDS
  	    	
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)


# add_executable(joint_highlevel_controller src/controller.cpp)
# target_link_libraries(joint_highlevel_controller ${catkin_LIBRARIES})
# add_dependencies(joint_highlevel_controller ${joint_kinematic_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(inverse_highlevel_controller src/controller_IK.cpp)
target_link_libraries(inverse_highlevel_controller 
${catkin_LIBRARIES}
${PINOCCHIO_LIBRARIES}
)
add_dependencies(inverse_highlevel_controller ${joint_kinematic_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(inverse_controllerA7 src/controllerA7.cpp)
target_link_libraries(inverse_controllerA7 
${catkin_LIBRARIES}
${PINOCCHIO_LIBRARIES}
)
add_dependencies(inverse_controllerA7 ${joint_kinematic_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(kinematic_client 
	src/client.cpp
  	src/ActionClient.cpp  
)

target_link_libraries(kinematic_client ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)