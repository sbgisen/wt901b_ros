#include "../include/serial_communication.hpp"
//[TODO] パスがJBDの方に飛んでしまう

////////////////////////////////////////////
//                                        //
//         serial_communication            //
//              2022/09/25                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////


// Start serial communication
// [USAGE] fd = uart_open("/dev/ttyXXX");
// [RETURN] Failture: -1
int Serial_Communication::uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("open %s success!\n",pathname);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 
    return fd; 
}

// Set bourate
// [USAGE] uart_set(fd,BAUD,8,'N',1)
// [RETURN] Failture: -1
int Serial_Communication::uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    // [Im NOT SURE...]
     struct termios newtio,oldtio; 
    if  ( tcgetattr( fd,&oldtio)  !=  0) {  
      perror("SetupSerial 1");
	  printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
      return -1; 
    }
    // [Im NOT SURE...]
    bzero( &newtio, sizeof( newtio ) ); 
    newtio.c_cflag  |=  CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  
    // [Im NOT SURE...]     
    switch( nBits ){ 
        case 7: 
        newtio.c_cflag |= CS7; 
        break; 
        case 8: 
        newtio.c_cflag |= CS8; 
        break; 
    } 
    // [Im NOT SURE...]     
    switch( nEvent ){ 
        case 'o':
        case 'O': 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag |= PARODD; 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        break; 
        case 'e':
        case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag &= ~PARODD; 
        break;
        case 'n':
        case 'N': 
        newtio.c_cflag &= ~PARENB; 
        break;
        default:
        break;
    } 

    // Set Baurate (Default is 9600)
    switch( nSpeed ){ 
        case 2400: 
            cfsetispeed(&newtio, B2400); 
            cfsetospeed(&newtio, B2400); 
            break; 
        case 4800: 
            cfsetispeed(&newtio, B4800); 
            cfsetospeed(&newtio, B4800); 
            break; 
        case 9600: 
            cfsetispeed(&newtio, B9600); 
            cfsetospeed(&newtio, B9600); 
            break; 
        case 115200: 
            cfsetispeed(&newtio, B115200); 
            cfsetospeed(&newtio, B115200); 
            break; 
        case 460800: 
            cfsetispeed(&newtio, B460800); 
            cfsetospeed(&newtio, B460800); 
            break; 
        default: 
            cfsetispeed(&newtio, B9600); 
            cfsetospeed(&newtio, B9600); 
            break; 
        } 
        if( nStop == 1 ) 
            newtio.c_cflag &=  ~CSTOPB; 
        else if ( nStop == 2 ) 
            newtio.c_cflag |=  CSTOPB; 
        newtio.c_cc[VTIME]  = 0; 
        newtio.c_cc[VMIN] = 0; 
        tcflush(fd,TCIFLUSH); 

    if((tcsetattr(fd,TCSANOW,&newtio))!=0){ 
        perror("com set error"); 
        return -1; 
    } 
    printf("set done!\n"); 
    return 0; 
}

// close serial communication
int Serial_Communication::uart_close(int fd)
{
    assert(fd);
    close(fd);
    return 0;
}

// [TODO] NOT WORKING
int Serial_Communication::send_data(int  fd, unsigned char *send_buffer,int length)
{
	int ret = write(fd,send_buffer,length*sizeof(unsigned char));
    printf("[ret]: %d",ret);
	return ret;
}

// [TODO] WHAT IS THIS
int Serial_Communication::recv_data(int fd, char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	return length;
}


// [TODO] Ping
int Serial_Communication::serial_ping(){
  return 0;
}

// [TODO] Reset FD
int Serial_Communication::serial_reconnect(){
  return 0;
}

int Serial_Communication::ParseData(char chr)
{
    static char chrBuf[100];
    static unsigned char chrCnt=0;
    signed short sData[4];
    unsigned char i;
    char cTemp=0;
    time_t now;
    chrBuf[chrCnt++]=chr;
    if (chrCnt<11) return -999;
    for (i=0;i<10;i++) cTemp+=chrBuf[i];
    if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);chrCnt--;return -999;}
    int height = -999;
    //chrBuf[2]-chrBuf[9] --copy to sData[0]-sData[3]
    memcpy(&sData[0],&chrBuf[2],8);
    // static int height_init;
    switch(chrBuf[1])
    {
            case 0x51:
                //accella
                for (i=0;i<3;i++) linear_acceleraion_a[i] = (float)sData[i]/32768.0*16.0*9.8;
                time(&now);
                ROS_DEBUG("T:%s Linear Acceleraion:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),linear_acceleraion_a[0],linear_acceleraion_a[1],linear_acceleraion_a[2]);                
                break;
            case 0x52:
                //angular(rad/s)
                for (i=0;i<3;i++) angular_velocity_w[i] = (float)sData[i]/32768.0*2000.0/180.0*3.141593;
                ROS_DEBUG("Angular Velocity:%7.3f %7.3f %7.3f ",angular_velocity_w[0],angular_velocity_w[1],angular_velocity_w[2]);					
                break;
            case 0x53:
                for (i=0;i<3;i++) angle[i] = (float)sData[i]/32768.0*3.141593;
                ROS_DEBUG("Angle:%7.3f %7.3f %7.3f ",angle[0],angle[1],angle[2]);
                break;
            case 0x54:
                for (i=0;i<3;i++) magnetic_h[i] = (float)sData[i]/100000000.0;
                temprature = (float)sData[3]/100.0;
                ROS_DEBUG("Magnetic:%4.4f %4.4f %4.4f , Tempra: %4.2f ",magnetic_h[0]*1000000,magnetic_h[1]*1000000,magnetic_h[2]*1000000, temprature);
                break;
            case 0x56:
                pressure = ((int)chrBuf[2]&0xFF) | ((int)chrBuf[3]&0xFF)<<8 | ((int)chrBuf[4]&0xFF)<<16 | ((int)chrBuf[5]&0xFF)<<24;
                height = ((int)chrBuf[6]&0xFF) | ((int)chrBuf[7]&0xFF)<<8 | ((int)chrBuf[8]&0xFF)<<16 | ((int)chrBuf[9]&0xFF)<<24;
                if(!init){
                    height_init = height;
                    init = true;
                }
                height_now = height;
                height = height - height_init;
                ROS_DEBUG("Press:%d , Height: %d ",pressure, height);
                break;
    }		
    chrCnt=0;
    return height; 		
}

int Serial_Communication::recv_loop(){
    ret = recv_data(fd,r_buf,44);
    if(ret == -1){
        fprintf(stderr,"uart read failed!\n");
        exit(EXIT_FAILURE);
    }
    int height_ret;
    for (int i=0;i<ret;i++) {
        height_ret = ParseData(r_buf[i]);
    }
    if(height_ret > -999){
        // msg.data = height_ret;
        printf("height: %d", height_ret);
        // pub_height.publish(msg);
    }
    return 0;
}

// Start Serial Communication
int Serial_Communication::serial_initialize(std::string port_name, int baurate)
{
    //return buffer -> 0 
    bzero(r_buf,1024);

    fd = uart_open(fd, port_name.c_str());
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        return 1;
    }

    if(uart_set(fd,baurate,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        return 1;
    }
    return 0;
}

Serial_Communication::Serial_Communication(){
    height_init = 0;
    height_now = 0;
    init = false;
}
Serial_Communication::~Serial_Communication(){
    ret = uart_close(fd);

    if(ret == -1){
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }
}
