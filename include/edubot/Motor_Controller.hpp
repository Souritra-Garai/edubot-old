#ifndef __MOTOR_CONTROLLER__
#define __MOTOR_CONTROLLER__

#include "math.h"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "PID_Calculator.hpp"

#define QUEUE_SIZE 10

class Motor_Controller: public PID_Calculator
{
    private:

        uint8_t target_PWM;
        std_msgs::UInt8 target_PWM_msg;

        bool target_direction;    
        std_msgs::Bool target_direction_msg;

        float current_angular_velocity;
        float target_angular_velocity;

        ros::Publisher target_PWM_pub;
        ros::Publisher target_direction_pub;

        ros::Subscriber current_angular_velocity_sub;
        ros::Subscriber target_angular_velocity_sub;

        void current_angular_velocity_callback(const std_msgs::Float32 &);
        void target_angular_velocity_callback(const std_msgs::Float32 &);
        
        void init_subscribers(ros::NodeHandle &, const char*);
        void init_publishers(ros::NodeHandle &, const char*);
        void init_variables();
        void init_messages();

        void ros_check_updates_and_publish();
        void update_motor_control_variables();

    public:

        Motor_Controller(ros::NodeHandle & nh);
        Motor_Controller(ros::NodeHandle & nh, const char* motorName);

        void spinOnce();
};

#endif