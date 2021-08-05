#ifndef __ROS_DIFFERENTIAL_DRIVE__
#define __ROS_DIFFERENTIAL_DRIVE__

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <edubot/SetFloatParam.h>

#include "edubot/DifferentialDrive.h"

class ROSDifferentialDrive
{
    private:

        DifferentialDrive &differential_drive_;

        ros::Subscriber cmdvel_sub_;
        void cmdvelCb_(const geometry_msgs::Twist &cmd_vel_msg);

        ros::Publisher left_wheel_ang_vel_pub_;
        std_msgs::Float32 left_wheel_ang_vel_msg_;
        
        ros::Publisher right_wheel_ang_vel_pub_;
        std_msgs::Float32 right_wheel_ang_vel_msg_;

        ros::ServiceServer set_max_wheel_speed_srv_server_;
        bool setMaxWheelSpeedSrvCb_(
            edubot::SetFloatParam::Request  &srv_rqst,
            edubot::SetFloatParam::Response &srv_resp
        );

        void init_pubs_(ros::NodeHandle &node_handle, std::string prefix);
        void init_subs_(ros::NodeHandle &node_handle, std::string prefix);
        void init_srv_servers_(ros::NodeHandle &node_handle, std::string prefix);
    
    public:
    
        ROSDifferentialDrive(
            DifferentialDrive &differential_drive,
            ros::NodeHandle &node_handle,
            std::string prefix
        );
};

#endif