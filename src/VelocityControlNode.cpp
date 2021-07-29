#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define WHEEL_BASE  0.212f // m
#define WHEEL_DIA   0.065f // m

#define WHEEL_MAX_VEL 12.0f // rad/s

void cmdVelSubscriderCallback(const geometry_msgs::Twist &message);
ros::Subscriber cmd_vel_subscriber;

std_msgs::Float32 left_motor_targ_ang_vel_msg;
std_msgs::Float32 right_motor_targ_ang_vel_msg;

ros::Publisher left_motor_targ_ang_vel_publisher;
ros::Publisher right_motor_targ_ang_vel_publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_control_node");

    ros::NodeHandle node_handle;

    left_motor_targ_ang_vel_publisher  = node_handle.advertise<std_msgs::Float32> ("left_motor/targ_ang_vel",  1000);
    right_motor_targ_ang_vel_publisher = node_handle.advertise<std_msgs::Float32> ("right_motor/targ_ang_vel", 1000);

    cmd_vel_subscriber = node_handle.subscribe("cmd_vel", 1000, cmdVelSubscriderCallback);

    ros::spin();

    return 0;
}

float clip(float velocity)
{
    return std::max(- WHEEL_MAX_VEL, std::min(WHEEL_MAX_VEL, velocity));
}

float getLeftWheelVelocity(float linear_x, float angular_z)
{
    float left_wheel_vel = linear_x - angular_z * WHEEL_BASE;
    
    left_wheel_vel /= WHEEL_DIA / 2;

    return clip(left_wheel_vel);
}

float getRightWheelVelocity(float linear_x, float angular_z)
{
    float right_wheel_vel = linear_x + angular_z * WHEEL_BASE;
    
    right_wheel_vel /= WHEEL_DIA / 2;

    return clip(right_wheel_vel);
}

void cmdVelSubscriderCallback(const geometry_msgs::Twist &message)
{
    left_motor_targ_ang_vel_msg.data  = getLeftWheelVelocity(message.linear.x, message.angular.z);
    right_motor_targ_ang_vel_msg.data = getRightWheelVelocity(message.linear.x, message.angular.z);

    left_motor_targ_ang_vel_publisher.publish(left_motor_targ_ang_vel_msg);
    right_motor_targ_ang_vel_publisher.publish(right_motor_targ_ang_vel_msg);
}
