#ifndef WT901B_ROS
#define WT901B_ROS

#include "../include/serial_communication.hpp"
#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<time.h>
#include<sys/types.h>
#include<errno.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64MultiArray.h>

#include <sensor_msgs/Imu.h>
// http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html

#include <sensor_msgs/MagneticField.h>
// http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html



std::vector<float> imu_data_(15,0.0);

/*
rosmsg show wt901b_ros/wt901b 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 magnetic_field
  float64 x
  float64 y
  float64 z
float64 temperature
float64 pressure
int32 height
*/

#endif //WT901B_ROS
