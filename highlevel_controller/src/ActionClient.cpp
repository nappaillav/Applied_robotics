#include "highlevel_controller/ActionClient.hpp"


ActionClient::ActionClient(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															move_to_action_client_ ("/gen3/action_planner/pose", true) {

	// if ( !readParameters() ) {
	// 	ROS_ERROR("Could not read parameters.");
	// 	ros::requestShutdown();
	// }
	if ( !node_handle_.getParam("/action_list/number_of_targets", num_tar )) {
		
		ROS_INFO("Cannot read tar_p" );
		// return false ;
	}
	ROS_INFO_STREAM("Num_target : "<< num_tar);
	// target_translation_ 	= Eigen::MatrixXd::Random(NUM_TARGETS_,3) ; // instead of random numbers, read it from YAML
	// feedback_topic_name_ 	= "/move_to_action/feedback" ; // instead of providing a string here, read it from YAML


	// prepare all publishers, subscribers, servers, clients, etc

	// feedback_subscriber_ = node_handle_.subscribe("/gen3/joint_states", 1, &ActionServer::feedbackCallback, this) ;
	// pose_subscriber_ = node_handle_.subscribe("/gen3/reference/pose", 1, &ActionServer::poseCallback, this) ;

	ROS_INFO_STREAM("Successfully launched node.") ;
}

ActionClient::~ActionClient() {

}

bool ActionClient::readParameters() {
	
	// node_handle_.param("/action_list/number_of_targets", num_tar);
	// ROS_INFO_STREAM(num_tar);
	return true;
}

void ActionClient::update() {

	ROS_INFO_STREAM("[ActionClient::update] Action client is ready. Wait for the server..." ) ;

	//boost::function fun_done = boost::bind(&ActionClient::doneCallback, this, _1, _2) ;

	for ( int counter = 0 ; counter < num_tar ; counter++ ) {
		
		// std::string action_name;
		orientation = "/action_list/action_" + std::to_string(counter) + "/orientation";
		translation = "/action_list/action_" + std::to_string(counter) + "/translation";
		duration = "/action_list/action_" + std::to_string(counter) + "/duration";
		
		node_handle_.param(translation, target_translation, std::vector<double>());
		node_handle_.param(orientation, target_orientation, std::vector<double>());
		// node_handle_.param(duration, target_time);
		node_handle_.getParam(duration, target_time );
		// ROS_INFO_STREAM("translation  : "<< target_translation[1]);
		// ROS_INFO_STREAM("time  : "<< target_time);
		// Fill in goal here
		
		action_goal_.x = target_translation[0];
		action_goal_.y = target_translation[1];
		action_goal_.z = target_translation[2];
		action_goal_.roll = target_orientation[0];
		action_goal_.pitch = target_orientation[1];
		action_goal_.yaw = target_orientation[2];
		action_goal_.T = target_time;

		// wait for the server
		move_to_action_client_.waitForServer();

		move_to_action_client_.sendGoal(action_goal_,
				boost::bind(&ActionClient::doneCallback, this, _1, _2),
				boost::bind(&ActionClient::activeCallback, this),
				boost::bind(&ActionClient::feedbackCallback, this, _1) ) ;

		ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
		move_to_action_client_.waitForResult(ros::Duration(30.0));

		if ( move_to_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
		}
		
	}


	ROS_INFO_STREAM("[ActionClient::doneCallback] Done, shotdown") ;
	ros::shutdown();
}



void ActionClient::doneCallback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) {
	ROS_INFO("[ActionClient::doneCallback] Finished in state [%s]", state.toString().c_str());
}

void ActionClient::activeCallback() {
	ROS_INFO_STREAM("[ActionClient::activeCallback] Action has become active");
}


void ActionClient::feedbackCallback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) {
	ROS_DEBUG_STREAM("[ActionClient::feedbackCallback]" << feedback->time_elapsed) ;
}