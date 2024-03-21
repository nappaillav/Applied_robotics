#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>
#include <cmath>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>

class ActionServer {

	public:
	/*!
	* Constructor.
	* @param node_handle_ the ROS node handle.
	*/
	ActionServer(ros::NodeHandle& node_handle);

	virtual ~ActionServer();

	void update() ;

	private:
	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	bool readParameters();

	void move_to_ballback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) ;
	void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg) ;
    void FeedbackDistance(); 
    
	ros::NodeHandle& node_handle_;

    // Task 1: Subscribe to the current position 
	ros::Subscriber feedback_subscriber_ ;

    // Task2 : Publisher
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;

	pinocchio::Model model;                                                 // create a model object
    pinocchio::Data data; 


    // Where is Service server being used 
	ros::ServiceServer serviceServer_;
    // Not necessary in Server
	highlevel_msgs::PoseCommandGoal action_goal_ ;

	actionlib::SimpleActionServer<highlevel_msgs::PoseCommandAction> move_to_action_server_ ;

	gazebo_msgs::ModelStates feedback_state_ ;

	highlevel_msgs::PoseCommandFeedback move_to_feedback_ ;

	ros::Rate loop_rate_ 	;

    Eigen::VectorXd target_pose;
    Eigen::VectorXd start_pose;
	Eigen::VectorXd joint_state;
    Eigen::VectorXd joint_vel;
	Eigen::VectorXd current_pose;
    Eigen::VectorXd current_rpy;
	Eigen::MatrixXd current_orientation;
	pinocchio::SE3 pose_now;

    double target_time, time_diff, T, st, vt;

	std::string urdf_file_name; 
	const int JOINT_ID = 7;

	Eigen::Quaterniond q_ini;
    Eigen::Quaterniond q_tar;
    Eigen::Quaterniond qs ;

	ros::Time start_time;
    ros::Time current_time;

	Eigen::VectorXd reference_pose;
	Eigen::VectorXd reference_velocity;

	geometry_msgs::Pose pose_msg;
	geometry_msgs::Twist twist_msg;
	bool ready;
	Eigen::VectorXd qs_rpy;

};

