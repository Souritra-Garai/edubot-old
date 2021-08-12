#ifndef __ROS_DIFFERENTIAL_DRIVE__
#define __ROS_DIFFERENTIAL_DRIVE__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <edubot/SetFloatParam.h>
#include <edubot/WheelAngularVelocityPair.h>

#include "edubot/DifferentialDrive.h"

class ROSDifferentialDrive : public DifferentialDrive 
{
    private:

        ros::Publisher vel_pub_;
        geometry_msgs::TwistStamped vel_msg_;
        
        ros::Publisher wheel_targ_ang_vel_pub_;
        edubot::WheelAngularVelocityPair wheel_targ_ang_vel_msg_;
        
        ros::Subscriber wheel_curr_ang_vel_sub_;
        void wheelCurrAngVelCb_(const edubot::WheelAngularVelocityPair &wheel_curr_ang_vel_msg);

        ros::Subscriber cmdvel_sub_;
        void cmdvelCb_(const geometry_msgs::Twist &cmd_vel_msg);
        
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
            ros::NodeHandle &node_handle,
            std::string prefix,
            float wheel_base,
            float wheel_radius,
            float max_wheel_speed = 1.0
        );
};

#endif