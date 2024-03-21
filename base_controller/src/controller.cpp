#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

class BaseController{
public:
    ros::NodeHandle nh; 
    ros::Subscriber twist_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber target_pose_sub;
    ros::Subscriber done_sub;
    ros::Publisher cmd_vel_pub;

    std_msgs::Bool done;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose target_pose;

    Eigen::Matrix2d jacobian;
    Eigen::Matrix2d jacobian_inverse;

    double roll, pitch, yaw;
    double phi;
    double husky_velocity_limit=1.0;

    geometry_msgs::Twist cmd_vel;
    Eigen::Vector2d desired_twist;
    Eigen::Vector2d result;
    
    BaseController(){
        nh = ros::NodeHandle("~");

        twist_sub = nh.subscribe("/planner/twist", 1, &BaseController::twistCallback, this);
        target_pose_sub = nh.subscribe("/planner/pose", 1, &BaseController::targetposeCallback, this);
        pose_sub = nh.subscribe("/husky_velocity_controller/feedback/pose", 1, &BaseController::poseCallback, this);
        done_sub = nh.subscribe("/planner/done", 1, &BaseController::doneCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
        
    }

    void targetposeCallback(const geometry_msgs::Pose::ConstPtr& tar_pose)
    {   
        // Update current pose
        target_pose = *tar_pose;
    }
    void poseCallback(const geometry_msgs::Pose::ConstPtr& cur_pose)
    {   
        // Update current pose
        current_pose = *cur_pose;
        tf2::Quaternion quaternion;
        tf2::fromMsg(current_pose.orientation, quaternion);
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        phi = yaw;

    }
    void doneCallback(const std_msgs::Bool::ConstPtr& done_msg) 
    {
        // done = done_msg->data;
        done = *done_msg;
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
    {
        if(!done.data)
        {
            desired_twist<<twist_msg->linear.x, twist_msg->linear.y;
            calculateJacobianInverse();

            result = jacobian_inverse * desired_twist;
            // Ensure the values do not exceed the velocity limit
            double v = result[0];
            double w = result[1];

            if (v > husky_velocity_limit) {
                v = husky_velocity_limit;
            }

            if (w > husky_velocity_limit) {
                w = husky_velocity_limit;
            }
            cmd_vel.linear.x = v;
            cmd_vel.angular.z = w;

        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

        }
        cmd_vel_pub.publish(cmd_vel);
        
    }

    void calculateJacobianInverse() 
    {
        // [[cos(φ ), − sin(φ )x_r − cos(φ )y_r]
        //,[sin(φ ), cos(φ )x_r − sin(φ )y_r]]
        jacobian << std::cos(phi), -std::sin(phi)*target_pose.position.x -std::cos(phi)*target_pose.position.y,
                    std::sin(phi), std::cos(phi)*target_pose.position.x -std::sin(phi)*target_pose.position.y;

        jacobian_inverse = jacobian.inverse();
    }
        
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller");
    BaseController controller;
    ros::spin();
    return 0;
}