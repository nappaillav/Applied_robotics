#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
class InverseKinematicController {

public:

    InverseKinematicController(ros::NodeHandle& nh) : nh(nh)
    {
        nh.getParam("/gen3/urdf_file_name", urdf_file_name);
        pinocchio::urdf::buildModel(urdf_file_name, model, false) ;
        data = pinocchio::Data(model) ;
         
        nq = model.nq;                                         // create the data structure for the calculations
        joint_state = Eigen::VectorXd::Zero(dim_joints);
        joint_vel = Eigen::VectorXd::Zero(dim_joints);                            // Should the velocity be zero
        target_state = Eigen::VectorXd::Zero(dim_joints);
        
        nh.param("/gen3/target/hand/position", default_positions, std::vector<double>());
        nh.param("/gen3/target/hand/orientation", default_orientation, std::vector<double>());
        
        trajectory_sub = nh.subscribe("/gen3/joint_states", 1, &InverseKinematicController::desiredTrajectoryCallback, this);

        pose_sub = nh.subscribe("/gen3/reference/pose", 1, &InverseKinematicController::poseCallback, this);
        // trajectory_sub = nh.subscribe("/gen3/reference/pose", 1, &InverseKinematicController::desiredTrajectoryCallback, this);


        joint_velocity_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 1);

        // move_to_service = nh.advertiseService("/pose_planner/move_to", &InverseKinematicController::moveToServiceCallback, this);
        // ori_to_service = nh.advertiseService("/pose_planner/move_ori", &InverseKinematicController::oriToServiceCallback, this);

        k = 1;  // Control gain
        flag = 0; // initalise with desired pose as target
        default_timer_flag = 0;

        // [TODO] Target orientation 
        target_pose = Eigen::VectorXd::Zero(6);
        target_pose(0) = default_positions[0];
        target_pose(1) = default_positions[1];
        target_pose(2) = default_positions[2];
        // [TODO] Orientation
        target_pose(3) = default_orientation[0];
        target_pose(4) = default_orientation[1];
        target_pose(5) = default_orientation[2];

        // initialization 
        joint_cmd.data.resize(dim_joints);
        current_pose = Eigen::VectorXd::Zero(3);
        // current_orientation = Eigen::MatrixXd::Zero(3,3);
        start_pose = Eigen::VectorXd::Zero(6);
        translation_ref = Eigen::VectorXd::Zero(3);
        vel = Eigen::VectorXd::Zero(6);
        qdot = Eigen::VectorXd::Zero(7);
        I = Eigen::MatrixXd::Identity(7,7);
        pose_err_vec_world = Eigen::VectorXd::Zero(6);
        jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints);
        vec2eigen();
        ready = false;
        ROS_INFO("HERE 1");
    }

    void vec2eigen(){
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

    void poseCallback(const geometry_msgs::Pose::ConstPtr& cur_pose)
    {   
        // Update current pose
        // current_pose = *cur_pose;
        flag = 1;
        translation_ref(0) = cur_pose->position.x;
        translation_ref(1) = cur_pose->position.y;
        translation_ref(2) = cur_pose->position.z;

        qs.x() = cur_pose->orientation.x;
        qs.y() = cur_pose->orientation.y;
        qs.z() = cur_pose->orientation.z;
        qs.w() = cur_pose->orientation.w;

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
        current_orientation = pose_now.rotation(); 
        current_rpy = pinocchio::rpy::matrixToRpy(pose_now.rotation());
        // ROS_INFO_STREAM("Here-D"<<current_rpy);

    }

    void update(){
        

        if (flag==1){
            
            // t = (current_time - start_time).toSec()/target_time;
            // translation_ref(0) = cubicPolynomial(start_pose(0), target_pose(0), t);
            // translation_ref(1) = cubicPolynomial(start_pose(1), target_pose(1), t);
            // translation_ref(2) = cubicPolynomial(start_pose(2), target_pose(2), t);

            vel(0) = k * (translation_ref(0) - current_pose(0))/dt;
            vel(1) = k * (translation_ref(1) - current_pose(1))/dt;
            vel(2) = k * (translation_ref(2) - current_pose(2))/dt;

            
            // qs = q_ini.slerp(t,q_tar);
            pose_ref.rotation() = qs.toRotationMatrix();
            pose_ref.translation() = translation_ref;
            

            pose_err = pose_now.inverse() * pose_ref;
            pose_err_vec_local = pinocchio::log6(pose_err).toVector();
            pose_err_vec_world.tail<3>() = pose_now.rotation() * pose_err_vec_local.tail<3>();

            vel(3) = k * pose_err_vec_world(3)/dt;
            vel(4) = k * pose_err_vec_world(4)/dt;
            vel(5) = k * pose_err_vec_world(5)/dt;

	        pinocchio::computeAllTerms(model, data, joint_state, joint_vel);
	        pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world);
            
            pseudo_inverse = jacobian_local_world.transpose() * (jacobian_local_world * jacobian_local_world.transpose()).inverse();

            qdot = pseudo_inverse * vel; // make a matrix
            joint_state = joint_state + qdot * dt + (I - pseudo_inverse * jacobian_local_world)*0.1*(target_state - joint_state); // publish this 
            // ROS_INFO_STREAM("Cur "<<pseudo_inverse * jacobian_dot);
            // ROS_INFO_STREAM("Cur "<<target_state);

            for(int i=0; i<dim_joints; i++){
                joint_cmd.data[i] = joint_state(i);
            }   
            joint_velocity_cmd_pub.publish(joint_cmd);
            
            current_time = ros::Time::now();

            // if (current_time >= end_time){
            //     flag = 0;
            //     ROS_INFO("TIME LIMIT REACHED");
            //     // ROS_INFO_STREAM("Cur "<<current_pose.transpose());
            // }
        }
        // else{
              
        //     joint_velocity_cmd_pub.publish(joint_cmd);
        // }
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber trajectory_sub;
    ros::Subscriber pose_sub;
    ros::Publisher joint_velocity_cmd_pub;
    ros::ServiceServer move_to_service;
    ros::ServiceServer ori_to_service;
    std_msgs::Float64MultiArray joint_cmd;
    std::vector<double> default_positions;
    std::vector<double> default_orientation;
    // Pinocchio model and data
    std::string urdf_file_name;

    const int JOINT_ID = 7;
    int dim_joints = 7;
    double dt = 0.002;
    pinocchio::SE3 pose_now;
    int nq;  
    double target_time;
    ros::Time start_time;
    ros::Time current_time;
    ros::Time end_time;
    Eigen::VectorXd target_pose;
    Eigen::VectorXd start_pose;

    pinocchio::Model model;                                                 // create a model object
    pinocchio::Data data; 
    Eigen::VectorXd joint_state;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd current_pose;
    Eigen::VectorXd current_rpy;
    Eigen::VectorXd target_state;
    Eigen::VectorXd vel;
    Eigen::VectorXd qdot;
    Eigen::VectorXd translation_ref;
    Eigen::MatrixXd I; 

    double x,y,z,t,k;
    Eigen::MatrixXd current_orientation;
    Eigen::MatrixXd jacobian_local_world;
    Eigen::MatrixXd jacobian_dot;
    Eigen::MatrixXd pseudo_inverse;
    
    Eigen::Quaterniond q_ini;
    Eigen::Quaterniond q_tar;
    Eigen::Quaterniond qs ;
    pinocchio::SE3 pose_ref;
    pinocchio::SE3 pose_err;
    Eigen::VectorXd pose_err_vec_local;
    Eigen::VectorXd pose_err_vec_world;
    
    int flag, translation, orientation;
    int default_timer_flag;
    bool ready;
};

int main(int argc, char** argv){
    
    ros::init(argc, argv, "inverse_highlevel_controllerA7");
    ros::NodeHandle nh("~");
    
    InverseKinematicController controller(nh);

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



