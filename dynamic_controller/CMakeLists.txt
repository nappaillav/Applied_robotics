cmake_minimum_required(VERSION 3.0.2)
project(dynamic_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  highlevel_msgs
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
  	DEPENDS
  	    	
)

include_directories(
	#include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)


add_executable(joint_dynamic_controller src/controller.cpp)
target_link_libraries(joint_dynamic_controller 
${catkin_LIBRARIES} 
${PINOCCHIO_LIBRARIES}
)
add_dependencies(joint_dynamic_controller ${joint_dynamic_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(inverse_dynamic_controller src/controller_IK.cpp)
target_link_libraries(inverse_dynamic_controller 
${catkin_LIBRARIES}
${PINOCCHIO_LIBRARIES}
)
add_dependencies(inverse_dynamic_controller ${joint_dynamic_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



