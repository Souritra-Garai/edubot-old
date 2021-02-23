#include "edubot/Motor_Controller.hpp"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh)
{
    init_messages();
    init_variables();
    init_publishers(nh, "/TPL", "/TDL");
}

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, const char* PWM, const char* DIR)
{
    init_messages();
    init_variables();
    init_publishers(nh, PWM, DIR);
}

void Motor_Controller::init_publishers(ros::NodeHandle &nh, const char* PWM, const char* DIR)
{
    target_PWM_pub = nh.advertise<std_msgs::UInt8>(PWM, QUEUE_SIZE);
    target_direction_pub = nh.advertise<std_msgs::Bool>(DIR, QUEUE_SIZE);
}

void Motor_Controller::init_variables()
{
    target_PWM = 0x00;
    target_direction = false;
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
    target_PWM += 0x01;
    target_direction ^= true;
}

void Motor_Controller::spinOnce()
{
    update_motor_control_variables();
    ros_check_updates_and_publish();
}