#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <highlevel_msgs/MoveTo.h>

class InverseDynamicController {

public:

    InverseDynamicController(ros::NodeHandle& nh) : nh(nh)
    {
        nh.getParam("/gen3/urdf_file_name", urdf_file_name);
        pinocchio::urdf::buildModel(urdf_file_name, model, false) ;
        data = pinocchio::Data(model) ;
         
        nq = model.nq;                                         // create the data structure for the calculations
        joint_state = Eigen::VectorXd::Zero(dim_joints);
        joint_vel = Eigen::VectorXd::Zero(dim_joints);                            // Should the velocity be zero
        target_state = Eigen::VectorXd::Zero(dim_joints);
        
        nh.param("/gen3/target/hand/position", default_positions, std::vector<double>());
        // nh.param("/gen3/target/joint/positions", target_joint_positions, std::vector<double>());
        
        trajectory_sub = nh.subscribe("/gen3/joint_states", 1, &InverseDynamicController::desiredTrajectoryCallback, this);

        joint_velocity_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 1);

        move_to_service = nh.advertiseService("/pose_planner/move_to", &InverseDynamicController::moveToServiceCallback, this);
        
        flag = 1; // initalise with desired pose as target
        default_timer_flag = 0;

        nh.param("/custom_IK/k_att", k_att, 3.0);
        nh.param("/custom_IK/stiffness", K, 100.0);
        nh.param("/custom_IK/damping", D, 30.0);

        nh.param("/custom_IK/k_0", K_0, 3.0);
        nh.param("/custom_IK/d_0", D_0, 0.5);

        target_pose.position.x = default_positions[0];
        target_pose.position.y = default_positions[1];
        target_pose.position.z = default_positions[2];

        // initialization 
        // stiffness = K * Eigen::MatrixXd::Identity(3,3);
        // damping = D * Eigen::MatrixXd::Identity(3,3);
        joint_cmd.data.resize(dim_joints);
        joint_state = Eigen::VectorXd::Zero(7);
        joint_vel = Eigen::VectorXd::Zero(7);
        current_pose = Eigen::VectorXd::Zero(3);
        pose_ref = Eigen::VectorXd::Zero(3);
        pose_acc = Eigen::VectorXd::Zero(3);
        pose_cmd = Eigen::VectorXd::Zero(3);

        vel = Eigen::VectorXd::Zero(3);

        I = Eigen::MatrixXd::Identity(7,7);
        jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints);
        jacobian_local_dot = Eigen::MatrixXd::Zero(6,dim_joints);
        vec2eigen();
        ready = false;
        ROS_INFO("HERE 1");
    }

    void vec2eigen(){
        // for (int i = 0; i < dim_joints; i++) {
        //     target_state(i) = target_joint_positions[i];
        // }
        target_state(0) = 1.57; 
        target_state(1) = 0.35; 
        target_state(2) = 1.57; 
        target_state(3) = -2.0; 
        target_state(4) = 0.00; 
        target_state(5) = -1.0;
        target_state(6) = 1.57;
    }

    bool isready(){
        return ready;
    }

    void desiredTrajectoryCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        
        // ROS_INFO("Sensor Data");
        ready = true;
        for (int i = 0; i < 7; i++) {
            joint_state(i) = msg->position[i];
            joint_vel(i) = msg->velocity[i];
        }

        pinocchio::forwardKinematics(model, data, joint_state, joint_vel);

        pose_now = data.oMi[JOINT_ID]; // end-effector pose w.r.t origin

        current_pose = pose_now.translation();

    }


    bool moveToServiceCallback(highlevel_msgs::MoveTo::Request& req, 
                               highlevel_msgs::MoveTo::Response& res) {
    
        target_pose.position.x = req.x;
        target_pose.position.y = req.y;
        target_pose.position.z = req.z;
        target_time = req.T;


        start_pose.position.x = current_pose(0);
        start_pose.position.y = current_pose(1);
        start_pose.position.z = current_pose(2);

        // ROS_INFO_STREAM("Target X "<<target_pose.position.x<<"Y "<<target_pose.position.y<<"Z "<<target_pose.position.z);
        // ROS_INFO_STREAM("starting X "<<start_pose.position.x<<"Y "<<start_pose.position.y<<"Z "<<start_pose.position.z);

        // condition for z < 0
        if (req.z < 0){
            res.out = false;   
            return false; 
        }
        
        start_time = ros::Time::now(); 
        end_time = start_time + ros::Duration(target_time); // private variable
        current_time = start_time;
        ROS_INFO("Starting the Service");
        flag = 1; // service started
        res.out = true;   
        return true;     
    }


    void update(){
        // update time if its default pos
        if(default_timer_flag==0){
            ROS_INFO("Defaut");
            start_pose.position.x = current_pose(0);
            start_pose.position.y = current_pose(1);
            start_pose.position.z = current_pose(2);
            start_time = ros::Time::now(); 
            target_time = 5.0;
            end_time = start_time + ros::Duration(target_time); // private variable
            current_time = ros::Time::now();
            default_timer_flag = 1;

        }

        if (flag==1){

            t = (current_time - start_time).toSec()/target_time;
            
            if (current_time >= end_time){
                // flag = 0;
                // ROS_INFO("TIME LIMIT REACHED");
                t = 1; 
                // ROS_INFO_STREAM("Cur "<<current_pose.transpose());
            }
            pose_ref(0) = cubicPolynomial(start_pose.position.x, target_pose.position.x, t);
            pose_ref(1) = cubicPolynomial(start_pose.position.y, target_pose.position.y, t);
            pose_ref(2) = cubicPolynomial(start_pose.position.z, target_pose.position.z, t);

            vel = k_att * (pose_ref - current_pose);

            // if (current_time >= end_time){
            //     vel = Eigen::VectorXd::Zero(3);
            // }

	        pinocchio::computeAllTerms(model, data, joint_state, joint_vel);
	        pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;
            pinocchio::getFrameJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_dot);

            jacobian = jacobian_local_world.topLeftCorner<3,7>();
            jacobian_dot = jacobian_local_dot.topLeftCorner<3,7>();

            M = data.M;
            v_fbk = jacobian * joint_vel; // end effector velocity 
            pseudo_inverse = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse(); //JT _1 == J-1 T

            P = I - (jacobian.transpose() * (jacobian* M.inverse()* jacobian.transpose()).inverse() * jacobian * M.inverse());
            tou_0 = -D_0 * joint_vel + K_0 * (target_state - joint_state);

            pose_acc = 18 * (vel - v_fbk);
            
            pose_cmd =  pose_acc + D * (vel - v_fbk) + K * (pose_ref - current_pose);

            // Have to compute jacobian_dot, add variables force, eta, lambda. 
            lambda = pseudo_inverse.transpose() * M * pseudo_inverse;
            eta = pseudo_inverse.transpose() * data.nle + lambda * jacobian_dot * joint_vel; // TODO : check q_dot
            force = lambda * pose_cmd + eta; 
            torque = jacobian.transpose() * force + P*tou_0;

            for(int i=0; i<dim_joints; i++){
                joint_cmd.data[i] = torque(i);
            }   
            joint_velocity_cmd_pub.publish(joint_cmd);
            
            current_time = ros::Time::now();
            
        }
    }

    double cubicPolynomial(double point_s, double point_f, double t)
    {
        double a0 = point_s;
        double a1 = 0.0;
        double a2 = 3*(point_f - point_s);
        double a3 = -2*(point_f - point_s);
        
        return a0 + a1*t + a2*t*t + a3*t*t*t;

        // t> T <-- (t==T) and t< T
    }
    double velcubicPolynomial(double point_s, double point_f, double t)
    {
        double a0 = point_s;
        double a1 = 0.0;
        double a2 = 3*(point_f - point_s);
        double a3 = -2*(point_f - point_s);
        
        return a1 + 2*a2*t + 3*a3*t*t;

        // t> T <-- (t==T) and t< T
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber trajectory_sub;
    ros::Publisher joint_velocity_cmd_pub;
    ros::ServiceServer move_to_service;
    std_msgs::Float64MultiArray joint_cmd;
    std::vector<double> default_positions;
    std::vector<double> target_joint_positions;
    // Pinocchio model and data
    std::string urdf_file_name;

    const int JOINT_ID = 7;
    int dim_joints = 7;
    double dt = 0.002;
    pinocchio::SE3 pose_now;
    int nq;  // Number of degrees of freedom
    double target_time;
    ros::Time start_time;
    ros::Time current_time;
    ros::Time end_time;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose start_pose;

    pinocchio::Model model;                                                 // create a model object
    pinocchio::Data data; 
    Eigen::VectorXd joint_state;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd current_pose;
    Eigen::VectorXd target_state;
    Eigen::VectorXd vel;
    Eigen::VectorXd pose_cmd;
    Eigen::VectorXd pose_acc;
    Eigen::VectorXd pose_ref;
    Eigen::VectorXd eta;
    Eigen::VectorXd force;
    Eigen::VectorXd torque;
    Eigen::VectorXd v_fbk;
    Eigen::VectorXd tou_0;

    Eigen::MatrixXd I; 

    double x,y,z,t,k_att, K, D, K_0, D_0;
    Eigen::MatrixXd jacobian_local_world;
    Eigen::MatrixXd jacobian_local_dot;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobian_dot;
    Eigen::MatrixXd pseudo_inverse;
    Eigen::MatrixXd lambda;
    Eigen::MatrixXd stiffness;
    Eigen::MatrixXd damping;
    Eigen::MatrixXd P;
    Eigen::MatrixXd M;

    int flag;
    int default_timer_flag;
    bool ready;
};

int main(int argc, char** argv){
    
    ros::init(argc, argv, "inverse_dynamic_controller");
    ros::NodeHandle nh("~");
    
    InverseDynamicController controller(nh);

    ros::Rate looprate(500);
    int count=0;

    while(ros::ok()){
        if(controller.isready()){
            controller.update();
        }
        // ROS_INFO_STREAM("Count : "<<count);
        ros::spinOnce();
        looprate.sleep();
        count++;
    }
    // ros::spin();
    return 0;
}


