#include "../include/wt901b_ros.hpp"
#include "wt901b_ros/wt901b.h"

//[TODO] FIX PATH

/*
Publish:
    /imu_height
    /imu_time
    /imu_linear_acceleraion
    /imu_angular_velocity
    /imu_angle
    /imu_magnetic
    /imu_temprature
    /imu_pressure
Subscribe:
    /imu_reset_height
*/

void resetHeghtCallback(const std_msgs::Bool::ConstPtr& msg){
    //height_init = height_now; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wt901b_ros");
    ros::NodeHandle nh("~");

    // Publisher
    ros::Publisher pub_imu = nh.advertise<wt901b_ros::wt901b>("/imu",1000);
    // ros::Publisher pub_height = nh.advertise<std_msgs::Int32>("/imu_height", 1000);
    // ros::Publisher pub_time = nh.advertise<std_msgs::Int32>("/imu_time", 1000);
    // ros::Publisher pub_linear_acceleraion = nh.advertise<std_msgs::Int32>("/imu_linear_acceleraion", 1000);
    // ros::Publisher pub_angular_velocity = nh.advertise<std_msgs::Int32>("/imu_angular_velocity", 1000);
    // ros::Publisher pub_angle = nh.advertise<std_msgs::Int32>("/imu_angle", 1000);
    // ros::Publisher pub_magnetic = nh.advertise<std_msgs::Int32>("/imu_magnetic", 1000);
    // ros::Publisher pub_temprature = nh.advertise<std_msgs::Int32>("/imu_temprature", 1000);
    // ros::Publisher pub_pressure = nh.advertise<std_msgs::Int32>("/imu_pressure", 1000);
    // Subscriber
    ros::Subscriber sub_reset_height = nh.subscribe<std_msgs::Bool>("/imu_reset_height", 1000, resetHeghtCallback);


    // launch WITHOUT .launch file
    std::string port_name = "/dev/ttyIMU";
    int baurate = 9600;
    // launch WITH .launch file
    nh.getParam("/wt901b_ros/port_name", port_name);
    nh.getParam("/wt901b_ros/baurate", baurate);
    ROS_INFO("param[%s], baurate[%d]", port_name.c_str(), baurate);

    // Start Serial Communication
    Serial_Communication serial_comminication;
    if(serial_comminication.serial_initialize(port_name, baurate)){
        ROS_ERROR("Could Not Open Serial [%s]", port_name.c_str());
        return 1;
    }
    ROS_INFO("Connected");
    

    //[TODO] add reset height
    // unsigned char reset_height[5] = {0xFF, 0xAA, 0x01, 0x03, 0x00};
    int height_ret;

    float linear_acceleraion_a[3];
    float angular_velocity_w[3];
    float angle[3];
    float magnetic_h[3];
    float temprature;
    int pressure;
    int height;

    while(ros::ok()){
        //updateImuData
        serial_comminication.recv_loop(imu_data_);
        
        //Substitude Data

        //Publish

        
        wt901b_ros::wt901b imu_data;
        imu_data.height = 1;
        pub_imu.publish(imu_data);

        // std_msgs::Int32 msg;

        // if(height_ret > -999){
        //     msg.data = height_ret;
        //     ROS_DEBUG("publish: %d", msg.data);
        //     pub_height.publish(msg);
        // }
        ros::spinOnce();

        usleep(1000);        
    }



    exit(EXIT_SUCCESS);
}
