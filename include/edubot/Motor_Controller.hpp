#ifndef __MOTOR_CONTROLLER__
#define __MOTOR_CONTROLLER__

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#define QUEUE_SIZE 10

class Motor_Controller
{
    private:

        uint8_t target_PWM;
        std_msgs::UInt8 target_PWM_msg;

        bool target_direction;    
        std_msgs::Bool target_direction_msg;

        ros::Publisher target_PWM_pub;
        ros::Publisher target_direction_pub;
        
        void init_publishers(ros::NodeHandle &, const char*, const char*);
        void init_variables();
        void init_messages();

        void ros_check_updates_and_publish();
        void update_motor_control_variables();

    public:

        Motor_Controller(ros::NodeHandle & nh);
        Motor_Controller(ros::NodeHandle & nh, const char* PWM_Topic, const char* Direction_Topic);

        void spinOnce();
};

#endif