cmake_minimum_required(VERSION 3.0.2)
project(cubic_polynomial_planner)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  gazebo_msgs
  std_msgs
  highlevel_msgs
  actionlib 			# need this to execute ROS action
)


catkin_package(
	INCLUDE_DIRS    
		include
	 	${EIGEN_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
    pinocchio
  	CATKIN_DEPENDS
    	roscpp   
    	actionlib 
      geometry_msgs
  	DEPENDS  		
)

include_directories(
	include
	${EIGEN_INCLUDE_DIRS}	
	${catkin_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}	
)

add_executable(cubic_polynomial_planner_server
	src/server.cpp
  src/ActionServer.cpp  
)

target_link_libraries(cubic_polynomial_planner_server ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

###########
## Build ##
###########


add_executable(cubic_polynomial_planner src/planner.cpp)
target_link_libraries(cubic_polynomial_planner ${catkin_LIBRARIES})
add_dependencies(cubic_polynomial_planner ${cubic_polynomial_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



