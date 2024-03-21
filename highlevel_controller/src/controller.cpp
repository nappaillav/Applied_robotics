#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>

class JointKinematicController {
public:
    JointKinematicController() {
        nh = ros::NodeHandle("~");

        target_joint_positions_sub = nh.subscribe("/gen3/joint_states", 1, &JointKinematicController::targetJointPositionsCallback, this);
        nh.param("/gen3/target/joint/positions", target_joint_positions, std::vector<double>());
        // ROS_INFO_STREAM("x="<<target_joint_positions[0]<<" y="<<target_joint_positions[1]<<" katt="<<target_joint_positions[6]);
        joint_position_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 1);

        k = 3;
    }

    
    void targetJointPositionsCallback(const sensor_msgs::JointState::ConstPtr& msg) {

        // std_msgs::Float64MultiArray control_commands;
        std::vector<double> control_commands(msg->position.size());

        // ROS_INFO("Here1");
        for (int i = 0; i < 7; i++) {
            control_commands[i] = msg->position[i] + k * (target_joint_positions[i] - msg->position[i])*0.02 ;
            // ROS_INFO_STREAM("val = "<<control_commands[i]);
        }
        // ROS_INFO("Here2");
        std_msgs::Float64MultiArray joint_cmd;
        joint_cmd.data = control_commands;

        joint_position_cmd_pub.publish(joint_cmd);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber target_joint_positions_sub;
    ros::Publisher joint_position_cmd_pub;

    std::vector<double> target_joint_positions;

    double k;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_kinematic_controller");
    JointKinematicController controller;

    ros::Rate loopRate(500) ;

    while ( ros::ok() ) {
        ros::spinOnce() ;
        loopRate.sleep() ;
    }
    
    ros::spin();
    return 0;
}
