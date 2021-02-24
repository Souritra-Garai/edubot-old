#include "edubot/Encoder_Reader.hpp"

Encoder_Reader::Encoder_Reader(ros::NodeHandle &nh)
{
    init_pubs_and_subs(nh, "");
    init_C_matrix();
}

Encoder_Reader::Encoder_Reader(ros::NodeHandle &nh, const char* motorName)
{
    init_pubs_and_subs(nh, motorName);
    init_C_matrix();
}

void Encoder_Reader::init_pubs_and_subs(ros::NodeHandle &nh, const char* motorName)
{
    std::string pub_topic(motorName);
    std::string sub_topic(motorName);

    pub_topic.append("/current_ang_vel");
    sub_topic.append("/E");

    angular_velocity_pub = nh.advertise<std_msgs::Float32>(pub_topic, QUEUE_SIZE);
    encoder_reading_sub = nh.subscribe(sub_topic, QUEUE_SIZE, &Encoder_Reader::encoder_reading_callback, this);
}

void Encoder_Reader::init_C_matrix()
{
    Matrix A_T;
    A_T << init_A_data;

    C = (A_T * A_T.transpose()).inverse() * A_T;
}

float Encoder_Reader::get_angular_velocity(Vector encoder_reading_vector)
{
    return 2 * PI * (C * encoder_reading_vector)[1] / 560;
}

void Encoder_Reader::encoder_reading_callback(const std_msgs::Int16MultiArray &msg)
{
    Vector encoder_reading_vector;

    for (int i=0; i<N; i++) encoder_reading_vector[i] = msg.data[i];

    angular_velocity_msg.data = get_angular_velocity(encoder_reading_vector);

    angular_velocity_pub.publish(angular_velocity_msg);
}