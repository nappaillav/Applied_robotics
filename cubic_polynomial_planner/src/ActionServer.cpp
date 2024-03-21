#include "cubic_polynomial_planner/ActionServer.hpp"


ActionServer::ActionServer(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															move_to_action_server_ (node_handle_, "/gen3/action_planner/pose", boost::bind(&ActionServer::move_to_ballback, this, _1 ), false),
															loop_rate_(500)
															{
														//	move_to_action_server_ (node_handle_, "go_to_home_configuration", false) {
	// if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
	// 	ros::console::notifyLoggerLevelsChanged();
	// }

	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// prepare all publishers, subscribers, servers, clients, etc
	feedback_subscriber_ = node_handle_.subscribe("/gen3/joint_states", 1, &ActionServer::feedbackCallback, this) ;
    
    // Publisher 
    pose_pub_ = node_handle_.advertise<geometry_msgs::Pose>("/gen3/reference/pose", 1);
    twist_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/gen3/reference/twist", 1);
    
    // Initialize the EIgen Vector
    start_pose = Eigen::VectorXd::Zero(6);
    target_pose = Eigen::VectorXd::Zero(6);
    joint_state = Eigen::VectorXd::Zero(7);
    joint_vel = Eigen::VectorXd::Zero(7);
    current_pose = Eigen::VectorXd::Zero(3);
    current_rpy = Eigen::VectorXd::Zero(3);
    qs_rpy = Eigen::VectorXd::Zero(3);
    reference_pose = Eigen::VectorXd::Zero(6);

	move_to_action_server_.start();
    ready = false;

	ROS_INFO_STREAM("[ActionServer::ActionServer] action server is ready");

}

ActionServer::~ActionServer() {

}


bool ActionServer::readParameters() {
	
    // TODO Define Variable
    node_handle_.getParam("/gen3/urdf_file_name", urdf_file_name);
    pinocchio::urdf::buildModel(urdf_file_name, model, false) ;
    data = pinocchio::Data(model) ;

	return true;
}
void ActionServer::FeedbackDistance(){
    // ROS_INFO("Feedback Distance");
    move_to_feedback_.distance_translation = std::sqrt(std::pow(target_pose(0) - reference_pose(0), 2)+
                                                    std::pow(target_pose(1) - reference_pose(1), 2)+
                                                    std::pow(target_pose(2) - reference_pose(2), 2));

    move_to_feedback_.distance_orientation = std::sqrt(std::pow(target_pose(3) - qs_rpy(0), 2)+
                                                    std::pow(target_pose(4) - qs_rpy(1), 2)+
                                                    std::pow(target_pose(5) - qs_rpy(2), 2));

    // move_to_feedback_.distance_orientation = std::sqrt(std::pow(q_tar.x() - qs.x(), 2)+
    //                                                 std::pow(q_tar.y() - qs.y(), 2)+
    //                                                 std::pow(q_tar.z() - qs.z(), 2)+
    //                                                 std::pow(q_tar.w() - qs.w(), 2));
}
void ActionServer::update() {
	
    if ( move_to_action_server_.isActive() ) {

        // ROS_INFO("HERE 1");
        current_time = ros::Time::now();
        time_diff = (current_time - start_time).toSec();
        if(time_diff > T){
            time_diff = T;
        }

        // ROS_INFO("HERE 2");
        move_to_feedback_.time_elapsed = time_diff;
        st = 3.0 * ((time_diff * time_diff)/(T*T)) - 2.0 * ((time_diff * time_diff * time_diff)/(T*T*T));
        vt = 6.0 * (time_diff/(T*T)) - 6.0 * ((time_diff * time_diff)/(T*T*T));
        // ROS_INFO_STREAM("ST: " << st);
        reference_pose = start_pose + st*(target_pose - start_pose);
        reference_velocity = vt*(target_pose - start_pose);

        // ROS_INFO("HERE 3");
        qs = q_ini.slerp(st,q_tar);


        pose_msg.position.x = reference_pose(0);
        pose_msg.position.y = reference_pose(1);
        pose_msg.position.z = reference_pose(2);
        pose_msg.orientation.x = qs.x();
        pose_msg.orientation.y = qs.y();
        pose_msg.orientation.z = qs.z();
        pose_msg.orientation.w = qs.w();

        pose_pub_.publish(pose_msg);
        // ROS_INFO("HERE 4");
        twist_msg.linear.x = reference_velocity(0);
        twist_msg.linear.y = reference_velocity(1);
        twist_msg.linear.z = reference_velocity(2);
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;

        twist_pub_.publish(twist_msg);

        // update the feedback
        move_to_feedback_.time_elapsed = time_diff;
        qs_rpy = pinocchio::rpy::matrixToRpy(qs.toRotationMatrix());
        FeedbackDistance();
    }

}

void ActionServer::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg) {

	
    // [TODO] Add to include
    // ROS_INFO("[ActionServer::feedbackCallback]");
    for (int i = 0; i < 7; i++) {
        joint_state(i) = msg->position[i];
        joint_vel(i) = msg->velocity[i];
    }
    pinocchio::forwardKinematics(model, data, joint_state, joint_vel);
    pose_now = data.oMi[JOINT_ID]; // end-effector pose w.r.t origin

    current_pose = pose_now.translation();
    current_orientation = pose_now.rotation(); 
    current_rpy = pinocchio::rpy::matrixToRpy(pose_now.rotation());
    ready = true;

	
}

void ActionServer::move_to_ballback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) {

	if(ready){
    ROS_INFO_STREAM("[ActionServer::move_to_ballback] goal: x=" << goal->x << ", y=" << goal->y << ", z= " << goal->z) ;
    start_pose(0) = current_pose(0);
    start_pose(1) = current_pose(1);
    start_pose(2) = current_pose(2);
    start_pose(3) = current_rpy(0);
    start_pose(4) = current_rpy(1);
    start_pose(5) = current_rpy(2);
    // [Done] start Position
    target_pose(0) = goal->x;
    target_pose(1) = goal->y;
    target_pose(2) = goal->z;
    target_pose(3) = goal->roll;
    target_pose(4) = goal->pitch;
    target_pose(5) = goal->yaw;
    T = goal->T;
    // ROS_INFO_STREAM("start_pose:"<<start_pose);
    // ROS_INFO_STREAM("target_pose:"<<target_pose);
    // ROS_INFO_STREAM("Target_time : "<<T);
    pinocchio::quaternion::assignQuaternion(q_tar, pinocchio::rpy::rpyToMatrix(target_pose(3),
                                                                            target_pose(4),
                                                                            target_pose(5)));

    pinocchio::quaternion::assignQuaternion(q_ini, pose_now.rotation());

    // [Done] Start Time
    // ROS_INFO("HERE 11");

    start_time = ros::Time::now();

    // [TODO] Update Time elapsed in Update
	move_to_feedback_.time_elapsed = 0.0;
    
    FeedbackDistance();
    qs_rpy(0) = current_rpy(0);
    qs_rpy(1) = current_rpy(1);
    qs_rpy(2) = current_rpy(2);
    // ROS_INFO_STREAM("Total Time "<< T);
	while (move_to_feedback_.time_elapsed < T) {
        
        // ROS_INFO("While loop : HERE 12");
		// publish the feedback. the client will be able to read it
		move_to_action_server_.publishFeedback(move_to_feedback_) ;

		// ROS_DEBUG_STREAM("[ActionServer::move_to_ballback] feedback=" << move_to_feedback_.distance) ;

		loop_rate_.sleep() ;
	}
    
    // if(move_to_feedback_.distance_translation < 0.01 && move_to_feedback_.distance_translation < 0.01)
    // {
    //     move_to_action_server_.setSucceeded() ;
    // }
	move_to_action_server_.setSucceeded() ;

	// ROS_INFO_STREAM("[ActionServer::move_to_ballback] Action is succesfully done") ;
    }
    else{
        ROS_INFO("Not ready");
    }
}
