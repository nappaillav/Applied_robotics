#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>

class JointDynamicController {
public:
    JointDynamicController(ros::NodeHandle& nh) : nh(nh) {

        nh.getParam("/gen3/urdf_file_name", urdf_file_name);
        pinocchio::urdf::buildModel(urdf_file_name, model, false) ;
        data = pinocchio::Data(model) ;

        sub = nh.subscribe("/gen3/joint_states", 1, &JointDynamicController::targetJointPositionsCallback, this);
        nh.param("/gen3/target/joint/positions", target_joint_positions, std::vector<double>());
        joint_position_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 1);

        nh.param("/custom/k_att", k_att, 3.0);
        nh.param("/custom/stiffness", K, 100.0);
        nh.param("/custom/damping", D, 30.0);

        ready = false;
        // Matrix 
        // stiffness = K * Eigen::MatrixXd::Identity(7,7);
        // damping = D * Eigen::MatrixXd::Identity(7,7);
        // Vector
        joint_cmd.data.resize(dim_joints);
        target_state = Eigen::VectorXd::Zero(dim_joints);
        q_ref = Eigen::VectorXd::Zero(dim_joints);
        v_ref = Eigen::VectorXd::Zero(dim_joints);
        a_ref= Eigen::VectorXd::Zero(dim_joints);
        torque = Eigen::VectorXd::Zero(dim_joints);
        q_cmd = Eigen::VectorXd::Zero(dim_joints);
        joint_state = Eigen::VectorXd::Zero(dim_joints);
        joint_velocity = Eigen::VectorXd::Zero(dim_joints);
        ROS_INFO_STREAM("K "<<K<<"D "<<D);
        target2vec();
        ROS_INFO("Here1");
    }
    bool isready(){
        return ready;
    }

    void target2vec(){
        for(int i=0; i<7; i++){
            target_state(i) = target_joint_positions[i];
        }
    }

    void targetJointPositionsCallback(const sensor_msgs::JointState::ConstPtr& msg) {

        // joint_position_cmd_pub.publish(joint_cmd);
        for (int i = 0; i < 7; i++) {
            joint_state(i) = msg->position[i];
            joint_velocity(i) = msg->velocity[i];
        }
        // feedback
        ready = true;
        
    }

    void update(){

        if(ready == true){
            // ROS_INFO_STREAM("target "<<target_state.transpose());
            // ROS_INFO_STREAM("joint_state "<<joint_state.transpose());
            v_ref = k_att * (target_state - joint_state); // vref
            // v_ref.normalize();
            // ROS_INFO_STREAM("V_Ref "<<v_ref.transpose());
            q_ref = joint_state + dt * v_ref; 
            // ROS_INFO_STREAM("q_ref "<<q_ref.transpose());
            // a_ref = (v_ref - joint_velocity)/dt; 
            // ROS_INFO_STREAM("a_ref "<<a_ref.transpose());
            // q_cmd = a_ref + K * (q_ref - joint_state) + D * (v_ref -  joint_velocity);
            q_cmd = a_ref + K* (q_ref - joint_state) - D * (joint_velocity);
            
            pinocchio::computeAllTerms(model, data, joint_state, joint_velocity);
            torque = data.M * q_cmd + data.nle;
            // torque.normalize();
            // ROS_INFO_STREAM("Torque "<<torque.transpose());
            // torque.normalize();
            for(int i=0; i<dim_joints; i++){
                joint_cmd.data[i] = torque(i);
            }   
            joint_position_cmd_pub.publish(joint_cmd);

        }

    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher joint_position_cmd_pub;
    pinocchio::Model model;                                                 // create a model object
    pinocchio::Data data;
    std::string urdf_file_name;
    std::vector<double> target_joint_positions;
    std_msgs::Float64MultiArray joint_cmd;
    double k_att, K, D;
    int dim_joints = 7;
    bool ready; 
    double dt = 0.002;
    Eigen::VectorXd joint_state;
    Eigen::VectorXd target_state;
    Eigen::VectorXd joint_velocity;
    Eigen::VectorXd q_ref;
    Eigen::VectorXd v_ref;
    Eigen::VectorXd a_ref;
    Eigen::VectorXd q_cmd;
    Eigen::VectorXd torque;
    Eigen::MatrixXd stiffness;
    Eigen::MatrixXd damping;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_Dynamic_controller");

    ros::NodeHandle nh("~");

    JointDynamicController controller(nh);

    ros::Rate looprate(500);

    while(ros::ok()){
        if(controller.isready()){
            // Check if ready needed
            controller.update();
        }
        ros::spinOnce();
        looprate.sleep();
    }
    // ros::spin();
    return 0;
}