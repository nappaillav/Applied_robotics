#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <highlevel_msgs/MoveTo.h>

class CubicPolynomialPlanner
{
public:
    // reference HelloTopicPublisher.hpp
    // I reference all variable in public 
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::ServiceServer service;
    ros::Publisher pub;
    geometry_msgs::Pose current_pose;
    ros::Time current_time;
    geometry_msgs::Pose target_pose;
    double target_time;

    CubicPolynomialPlanner()
    {   
        nh = ros::NodeHandle("~");  // this initialization helps node initialization and service path name

        // Subscribe to Robot's current Position
        sub = nh.subscribe("/firefly/ground_truth/pose", 1, &CubicPolynomialPlanner::poseCallback, this);
        
        // Advertice the Service
        // I had to define /pose_planner/move_to, without NodeHandle("~")
        service = nh.advertiseService("move_to", &CubicPolynomialPlanner::moveToServiceCallback, this);
        
        // Advertise the Pose to /firefly/command/pose
        pub = nh.advertise<geometry_msgs::Pose>("/firefly/command/pose", 1);
        
    }
    
    // in turtlesim draw_square.cpp they use turtlesim::PoseConstPtr g_pose;
    void poseCallback(const geometry_msgs::Pose::ConstPtr& cur_pose)
    {   
        // Update current pose as well as the time 
        current_pose = *cur_pose;
    }

    bool moveToServiceCallback(highlevel_msgs::MoveTo::Request& req,
                               highlevel_msgs::MoveTo::Response& res)
    {
        if(req.z <= 0.0){
            ROS_ERROR("Target position is not Possible");
            res.out = false;
            return false;
        }

        target_pose.position.x = req.x;
        target_pose.position.y = req.y;
        target_pose.position.z = req.z;

        target_time = req.T;

        // ros::Time start_time = current_time;
        ros::Time start_time = ros::Time::now();

        ros::Time end_time = start_time + ros::Duration(target_time);

        ros::Time current_time = start_time;

        // loop rate : http://wiki.ros.org/roscpp/Overview/Time
        // example : https://roboticsbackend.com/ros-rate-roscpy-roscpp/
        ros::Rate rate(500); // 500Hz
        // ROS_INFO("Starting ");
        while(ros::ok() && current_time < end_time)
        {
            double t = (current_time - start_time).toSec()/target_time;
            double x = cubicPolynomial(current_pose.position.x, target_pose.position.x, t);
            double y = cubicPolynomial(current_pose.position.y, target_pose.position.y, t);
            double z = cubicPolynomial(current_pose.position.z, target_pose.position.z, t);

            // Do I need to initialize everytime inside
            geometry_msgs::Pose new_pose;

            new_pose.position.x = x;
            new_pose.position.y = y;
            new_pose.position.z = z;

            // publishes the new pose
            pub.publish(new_pose);
            
            rate.sleep();
            
            // do i need to update the time even if I'm 
            // Part:1 Comment this one and use the current_time from poseCallback
            // Part:2 Comment the current time in poseCallback []
            current_time = ros::Time::now();
        }
        res.out = true;
        // ROS_INFO("Ending ");
        return true;
        
    }

    double cubicPolynomial(double point_s, double point_f, double t)
    {
        double a0 = point_s;
        double a1 = 0.0;
        double a2 = 3*(point_f - point_s);
        double a3 = -2*(point_f - point_s);
        // Derivation is "03-trajectory.pdf" Slide : 21
        return a0 + a1*t + a2*t*t + a3*t*t*t;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cubic_polynomial_planner");
    CubicPolynomialPlanner node;
    ros::spin();
    return 0;
}