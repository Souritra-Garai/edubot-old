/**
 * @file ROSMotorController.h
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief 
 * @version 0.1
 * @date 2021-07-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __ROS_MOTOR_CONTROLLER__
#define __ROS_MOTOR_CONTROLLER__

#include "ros.h"
#include "std_msgs/Float32.h"

#include "MotorController.h"

class ROSMotorController : public MotorController
{
    private :

        char target_angular_velocity_subscriber_topic_[20];
        char current_angular_velocity_publisher_topic_[20];
        
        ros::Subscriber<std_msgs::Float32, ROSMotorController> target_angular_velocity_subscriber_;
        
        std_msgs::Float32 current_angular_velocity_message_;
        ros::Publisher current_angular_velocity_publisher_;

        const char * getTargetAngularVelocityTopic_(const char *);
        const char * getCurrentAngularVelocityTopic_(const char *);

    public :

        ROSMotorController(
            const char * topic_prefix,
            uint8_t direction_pin,
            uint8_t encoder_pin_1,
            uint8_t encoder_pin_2,
            float update_frequency,
            float counts_per_rotation
        );

        void target_angular_velocity_callback(const std_msgs::Float32 &target_velocity_message);

        void initialize_publishers_and_subscribers(ros::NodeHandle &node_handle);

        void publish();
};

#endif