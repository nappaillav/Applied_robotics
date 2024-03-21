#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::Publisher pub;

void keyboardInput(const std_msgs::String::ConstPtr& msg)
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    ROS_INFO("I heard: [%s]", msg->data.c_str());

    if(msg->data == "i"){
        twist.linear.x = 0.5; // Move Straight
    }
    
    else if(msg->data == "u"){
        // Move Left
        twist.linear.x = 0.5;
        twist.angular.z = 0.5;
    }
    
    else if(msg->data == "o"){
        // Move Right
        twist.linear.x = 0.5;
        twist.angular.z = -0.5;
    }
    // Publishes the twist corresponding to the Keyboard input
    pub.publish(twist);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_controller");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/teleop/cmd", 1000, keyboardInput);

    ros::spin();
    return 0;
}