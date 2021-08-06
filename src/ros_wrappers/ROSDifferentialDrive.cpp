#include "ros_wrappers/ROSDifferentialDrive.h"

ROSDifferentialDrive::ROSDifferentialDrive(
    DifferentialDrive &differential_drive,
    ros::NodeHandle &node_handle,
    std::string prefix
) : differential_drive_(differential_drive)
{
    init_pubs_(node_handle, prefix);
    init_subs_(node_handle, prefix);
    init_srv_servers_(node_handle, prefix);
}

void ROSDifferentialDrive::init_subs_(
    ros::NodeHandle &node_handle,
    std::string prefix
) {
    cmdvel_sub_ = node_handle.subscribe(prefix + "cmd_vel", 1000, &ROSDifferentialDrive::cmdvelCb_, this);
}

void ROSDifferentialDrive::init_pubs_(
    ros::NodeHandle &node_handle,
    std::string prefix
) {
    left_wheel_ang_vel_pub_  = node_handle.advertise<std_msgs::Float32> (prefix + "left_motor/targ_ang_vel",  1000);
    right_wheel_ang_vel_pub_ = node_handle.advertise<std_msgs::Float32> (prefix + "right_motor/targ_ang_vel", 1000);
}

void ROSDifferentialDrive::init_srv_servers_(
    ros::NodeHandle &node_handle,
    std::string prefix
) {
    set_max_wheel_speed_srv_server_ = node_handle.advertiseService(
        prefix + "set_max_wheel_speed",
        &ROSDifferentialDrive::setMaxWheelSpeedSrvCb_,
        this
    );
}

bool ROSDifferentialDrive::setMaxWheelSpeedSrvCb_(
    edubot::SetFloatParam::Request  &srv_rqst,
    edubot::SetFloatParam::Response &srv_resp
) {
    differential_drive_.setMaxWheelSpeed(srv_rqst.val);
    return true;
}

void ROSDifferentialDrive::cmdvelCb_(const geometry_msgs::Twist &msg)
{
    std::pair<float, float> wheel_vel = differential_drive_.getWheelVelocity(msg.linear.x, msg.angular.z);

    left_wheel_ang_vel_msg_.data  = wheel_vel.first;
    right_wheel_ang_vel_msg_.data = wheel_vel.second;

    left_wheel_ang_vel_pub_.publish(left_wheel_ang_vel_msg_);
    right_wheel_ang_vel_pub_.publish(right_wheel_ang_vel_msg_);
}