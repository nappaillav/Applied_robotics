#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <vector>


class ActionClient {

	public:
		/*!
		* Constructor.
		* @param node_handle_ the ROS node handle.
		*/
		ActionClient(ros::NodeHandle& node_handle);

		/*!
		 * Destructor.
		 */
		virtual ~ActionClient();

		/*
		 * A function handling necessary actions in every loop
		 */
		void update() ;

	protected:

		void activeCallback() ;
		void feedbackCallback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) ;
		void doneCallback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) ;

	private:
		/*!
		* Reads and verifies the ROS parameters.
		* @return true if successful.
		*/
		bool readParameters();

		//! ROS node handle.
		ros::NodeHandle& node_handle_;

		//! ROS topic subscriber.
		ros::Subscriber feedback_subscriber_ ;

		//! ROS topic name to subscribe to.
		// std::string feedback_topic_name_ ;

		//! ROS service server.
		ros::ServiceServer serviceServer_;

		highlevel_msgs::PoseCommandGoal action_goal_ ;

		actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> move_to_action_client_ ; //("go_to_home_configuration", true); // true -> don't need ros::spin()

		int num_tar;
		Eigen::MatrixXd target_translation_ ;

		int counter_ = 0 ;
		
		std::string orientation;

		std::string translation;

		std::string duration;

		std::vector<double> target_translation;
    	std::vector<double> target_orientation;
		int target_time;
};

