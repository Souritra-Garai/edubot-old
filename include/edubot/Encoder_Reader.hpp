#ifndef __ENC_READER__
#define __ENC_READER__

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"

#include <eigen3/Eigen/Dense>

#ifndef QUEUE_SIZE
#define QUEUE_SIZE 10
#endif

#define N 5
#define M 2
#define PI 3.14159
#define init_A_data  1.0, 1.0, 1.0, 1.0, 1.0, 0.001, 0.002, 0.003, 0.004, 0.005

typedef Eigen::Matrix<float, M, N> Matrix;
typedef Eigen::Matrix<float, N, 1> Vector;

class Encoder_Reader
{
    private:

        Matrix C;

        std_msgs::Float32 angular_velocity_msg;

        ros::Publisher angular_velocity_pub;
        ros::Subscriber encoder_reading_sub;

        void init_pubs_and_subs(ros::NodeHandle &nh, const char* motorName);
        void init_C_matrix();

    public:

        Encoder_Reader(ros::NodeHandle &nh);
        Encoder_Reader(ros::NodeHandle &nh, const char* motorName);

        void encoder_reading_callback(const std_msgs::Int16MultiArray &msg);
        float get_angular_velocity(Vector encoder_reading_vector);
};

#endif