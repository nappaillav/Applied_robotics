#include <highlevel_controller/ActionClient.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "action_client") ;
	ros::NodeHandle node_handle ;

	ActionClient action_client(node_handle) ;

	ros::Rate loop_rate(500) ;

	while ( ros::ok() ) {
		ros::spinOnce();
		action_client.update() ;
		loop_rate.sleep() ;
	}

	return 0;
}