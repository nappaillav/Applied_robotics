
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <cmath>

class PotentialFieldPlanner {
public:
    ros::NodeHandle nh; 
    ros::Subscriber sub;
    geometry_msgs::Pose current_pose;
    ros::ServiceServer service;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;
    ros::Publisher done_pub;

    double target_x;
    double target_y;
    double katt;
    double max_linear_velocity;
    double max_angular_velocity;

    double robot_x;
    double robot_y;
    double dx, dy, dt, distance;
    int flag = 0;

    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    std_msgs::Bool done;
    double c_time;
    double s_time;
    double max_velocity, scale;

    PotentialFieldPlanner(ros::NodeHandle& nh) : nh(nh) {

        // nh = ros::NodeHandle("~");  // this initialization helps node initialization and service path name
        done.data = false;
        // Read target position and katt parameter from ROS parameter server
        nh.param("/target/x", target_x, 0.0);
        nh.param("/target/y", target_y, 0.0);
        nh.param("/planner/k_att", katt, 10.0);
        nh.param("max_linear_velocity", max_linear_velocity, 1.0);
        nh.param("max_angular_velocity", max_angular_velocity, 1.0);
        
        // ROS_INFO_STREAM("x="<<target_x<<" y="<<target_y<<" katt="<<katt<<" P="<<max_linear_velocity<<" Y="<<max_angular_velocity);
        // Initialize publishers and service
        sub = nh.subscribe("/husky_velocity_controller/feedback/pose", 1, &PotentialFieldPlanner::poseCallback, this);
        service = nh.advertiseService("start", &PotentialFieldPlanner::startCallback, this);

        pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1);
        twist_pub = nh.advertise<geometry_msgs::Twist>("twist", 1);
        done_pub = nh.advertise<std_msgs::Bool>("done", 1);

    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& cur_pose)
    {   
        // Update current pose
        current_pose = *cur_pose;
    }
    void update(){
        c_time = ros::Time::now().toSec();
        if (flag == 1){
            robot_x = current_pose.position.x; 
            robot_y = current_pose.position.y;

            dt = c_time - s_time;

            dx = target_x - robot_x;
            dy = target_y - robot_y;
            distance = std::sqrt(dx * dx + dy * dy);

            pose.position.x = dt * twist.linear.x;
            pose.position.y = dt * twist.linear.y;

            if (distance < 0.01) 
            {
                // Robot has arrived at the target
                done.data = true;
                
            } 
            else 
            {
                // Calculate the linear velocity 
                max_velocity = max_linear_velocity; // Set your maximum velocity here
                scale = max_velocity / (katt*distance);

                // TODO : if velocity is more, 
                if(katt*distance > max_linear_velocity){
                    twist.linear.x = (katt * dx) * scale;
                    twist.linear.y = (katt * dy) * scale;
                }
                else
                {
                    twist.linear.x = katt * dx;
                    twist.linear.y = katt * dy;
                }

                
                done.data = false;
            }
            s_time = c_time;
            // Publish pose, twist, and done status
            pose_pub.publish(pose);
            twist_pub.publish(twist);
            done_pub.publish(done);


        }

    }
    bool startCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

        // ROS_INFO("Start");

        ROS_INFO_STREAM("Trigger");
        flag = 1;
        res.success = true;
        // ROS_INFO("End");
        s_time = ros::Time::now().toSec();
        res.message = "Started";
        return true;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "potential_field_planner");
    
    // PotentialFieldPlanner planner;

    ros::NodeHandle nh("~") ;
    PotentialFieldPlanner planner(nh);
    ros::Rate loopRate(500) ;
    
    //ROS_INFO("Server: a server is created");

    while ( ros::ok() ) {
        // call all the callbacks waiting to be called
        planner.update() ;
        ros::spinOnce() ;
        loopRate.sleep() ;
    }
    
    ros::spin();
    return 0;
}
