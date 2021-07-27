/**
 * @file ROSMotorController.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief 
 * @version 0.1
 * @date 2021-07-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ROSMotorController.h"

// Constructor
ROSMotorController::ROSMotorController(
    const char * topic_prefix,
    uint8_t direction_pin,
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation
) : MotorController(
        direction_pin,
        encoder_pin_1,
        encoder_pin_2,
        update_frequency,
        counts_per_rotation
    ),
    target_angular_velocity_subscriber_(
        getTargetAngularVelocityTopic_(topic_prefix),
        &ROSMotorController::target_angular_velocity_callback,
        this
    ),
    current_angular_velocity_publisher_(
        getCurrentAngularVelocityTopic_(topic_prefix),
        &current_angular_velocity_message_
    )
{ ;}

void ROSMotorController::target_angular_velocity_callback(const std_msgs::Float32 &target_velocity_message)
{
    setTargetStateValue(target_velocity_message.data);
}

const char * ROSMotorController::getTargetAngularVelocityTopic_(const char * topic_prefix)
{
    strcpy(target_angular_velocity_subscriber_topic_, topic_prefix);
    return strcat(target_angular_velocity_subscriber_topic_, "/targ_ang_vel");
}

const char * ROSMotorController::getCurrentAngularVelocityTopic_(const char * topic_prefix)
{
    strcpy(current_angular_velocity_publisher_topic_, topic_prefix);
    return strcat(current_angular_velocity_publisher_topic_, "/curr_ang_vel");
}

void ROSMotorController::initialize_publishers_and_subscribers(ros::NodeHandle &node_handle)
{
    node_handle.subscribe(target_angular_velocity_subscriber_);
    node_handle.advertise(current_angular_velocity_publisher_);
}

void ROSMotorController::publish()
{
    current_angular_velocity_message_.data = getMotorAngularVelocity();
    current_angular_velocity_publisher_.publish(&current_angular_velocity_message_);
}