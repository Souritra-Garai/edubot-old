#include "edubot/Motor_Controller.hpp"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh) : PID_Calculator()
{
    init_messages();
    init_variables();
    init_publishers(nh, "");
    init_subscribers(nh, "");
}

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, const char* motorName)
{
    init_messages();
    init_variables();
    init_publishers(nh, motorName);
    init_subscribers(nh, motorName);
}

void Motor_Controller::init_publishers(ros::NodeHandle &nh, const char* motorName)
{
    std::string PWM_Topic(motorName);
    std::string Dir_Topic(motorName);
    
    PWM_Topic.append("/P");
    Dir_Topic.append("/D");

    target_PWM_pub = nh.advertise<std_msgs::UInt8>(PWM_Topic, QUEUE_SIZE);
    target_direction_pub = nh.advertise<std_msgs::Bool>(Dir_Topic, QUEUE_SIZE);
}

void Motor_Controller::init_subscribers(ros::NodeHandle &nh, const char* motorName)
{
    std::string CAV_Topic(motorName);
    std::string TAV_Topic(motorName);

    CAV_Topic.append("/current_ang_vel");
    TAV_Topic.append("/target_ang_vel");

    current_angular_velocity_sub = nh.subscribe(CAV_Topic, QUEUE_SIZE, &Motor_Controller::current_angular_velocity_callback, this);
    target_angular_velocity_sub = nh.subscribe(TAV_Topic, QUEUE_SIZE, &Motor_Controller::target_angular_velocity_callback, this);
}

void Motor_Controller::init_variables()
{
    target_PWM = 0x00;
    target_direction = false;

    target_angular_velocity = 0.0;
    current_angular_velocity = 0.0;
}

void Motor_Controller::init_messages()
{
    target_PWM_msg.data = 0x00;
    target_direction_msg.data = false;
}

void Motor_Controller::ros_check_updates_and_publish()
{
    if (target_direction_msg.data ^ target_direction)
    {
        target_direction_msg.data = target_direction;
        target_direction_pub.publish(target_direction_msg);
    }

    if (target_PWM_msg.data ^ target_PWM)
    {
        target_PWM_msg.data = target_PWM;
        target_PWM_pub.publish(target_PWM_msg);
    }
}

void Motor_Controller::update_motor_control_variables()
{
    float val = 255.0 * tanh(calc_effort(target_angular_velocity - current_angular_velocity));
    
    target_PWM = (uint8_t) abs(val);
    target_direction = signbit(val);
}

void Motor_Controller::spinOnce()
{
    update_motor_control_variables();
    ros_check_updates_and_publish();
}

void Motor_Controller::current_angular_velocity_callback(const std_msgs::Float32 &msg)
{
    current_angular_velocity = msg.data;
    // printf("Updated current angular velocity to %f\n", current_angular_velocity);
}

void Motor_Controller::target_angular_velocity_callback(const std_msgs::Float32 &msg)
{
    target_angular_velocity = msg.data;
    // printf("Updated target angular velocity to %f\n", target_angular_velocity);
}