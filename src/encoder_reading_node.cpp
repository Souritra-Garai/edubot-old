#include "ros/ros.h"

#include "edubot/Encoder_Reader.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Encoder_Reading_Node");

    ros::NodeHandle node_handle; 

    // printf("Encoder Reading Node initialized\n");

    Encoder_Reader encoder_L(node_handle, "U1/L");
    Encoder_Reader encoder_R(node_handle, "U1/R");

    // printf("Motor object initialized\n");

    ros::spin();
    
    return 0;
}