#ifndef SERIAL_COMMUNICATION
#define SERIAL_COMMUNICATION

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
#include<vector>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64MultiArray.h>
#define SERIAL_PORT "/dev/ttyIMU"
#define BAUD 9600

static int ret;
static int fd;


class Serial_Communication{

    private:

        std::vector<float> imu_data_temp_(15,0.0);
        enum Params {
            LINEAR_ACCEL_X,
            LINEAR_ACCEL_Y,
            LINEAR_ACCEL_Z,
            ANGULAR_VEL_X,
            ANGULAR_VEL_Y,
            ANGULAR_VEL_Z,
            ANGLE_X,
            ANGLE_Y,
            ANGLE_Z,
            MAGNETIC_X,
            MAGNETIC_Y,
            MAGNETIC_Z,
            TEMPLATURE,
            PRESSURE,
            HEIGHT
        };
        
    public:
        // /imu_height
        // /imu_time
        // /imu_linear_acceleraion
        // /imu_angular_velocity
        // /imu_angle
        // /imu_magnetic
        // /imu_temprature
        // /imu_pressure        

        float linear_acceleraion_a[3];
        float angular_velocity_w[3];
        float angle[3];
        float magnetic_h[3];
        float temprature;
        int pressure;
        int height;

        int height_init;
        int height_now;
        bool init;
        char r_buf[1024];

        Serial_Communication();
        ~Serial_Communication();
        int serial_initialize(std::string port_name, int baurate);

        int send_data(int  fd, unsigned char *send_buffer,int length);
        int recv_data(int fd, char* recv_buffer,int length);
        int ParseData(char chr);
        int recv_loop(std::vector<float>& battery_status);

        int uart_open(int fd,const char *pathname);
        int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop);
        int uart_close(int fd);
        // [TODO]
        int serial_ping();
        int serial_reconnect();


};


#endif //SERIAL_COMMUNICATION
