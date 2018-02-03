#include <ros/ros.h> 
#include <serial/serial.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <geometry_msgs/Twist.h>

serial::Serial ser; 

char stop[9]={0xA5,0x5A,0x07,0XA1,0,0,0,0xA8,0xAA};
//char buffer[9]={0xA5,0x5A,0x07,0XA1,0,0,0,0xA8,0xAA};
void write_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{ 
	std::vector<uint8_t> data;
	//std_msgs::String::ConstPtr& vel;
	//vel->data=0;
    //char buffer[9]={0xA5,0x5A,0x07,0XA1,2,2,2,0xAE,0xAA};
    //A5 5A 是帧头 AA是尾，07除帧头以外的字节数，A1，标识，不改，data1，转向，data2，右轮，data3，左轮。
    // data只能是1,2,4.1是停止，2是前进，4是后退  AA是校验和，倒数第二字节
    char buffer[9]={0xA5,0x5A,0x07,0XA1,1,4,1,0xAA,0xAA};
    //buffer[4]=static_cast<char>(msg->angular.z);//C++的强制类型转换和C的不一样
    //buffer[5]=static_cast<char>(msg->linear.x);
   // buffer[6]=static_cast<char>(msg->linear.x)；

    if(msg->angular.z==2)  buffer[4]=2, buffer[5]=1, buffer[6]=1;
    if(msg->angular.z==-2) buffer[4]=4, buffer[5]=1, buffer[6]=1;
    if(msg->linear.x==2)  buffer[4]=1, buffer[5]=2, buffer[6]=1;
    if(msg->linear.x==-2)  buffer[4]=1, buffer[5]=4, buffer[6]=1;
    buffer[7]=0x07+0xA1+buffer[4] + buffer[5] + buffer[6];//校验和

    ser.write(buffer);
	ROS_INFO_STREAM("Keyboard information z" << msg->angular.z);
	ROS_INFO_STREAM("Keyboard information x" << msg->linear.x);    
 
	ROS_INFO_STREAM("Keyboard information z" << (char)buffer[4]);    
	ROS_INFO_STREAM("Keyboard information x" << (char)buffer[5]); 
    ROS_INFO_STREAM("buffer[6]" << (int)buffer[6]);   
    ROS_INFO_STREAM("sum" << (int)buffer[7]);   
} 

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //订阅主题，并配置回调函数 
    ros::Subscriber write_sub = nh.subscribe("turtle1/cmd_vel", 1000, write_callback); 
    //发布主题 
   // ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(9600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
  
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    while(ros::ok()) 
    { 
  
        if(ser.available()){ 
            
            //ROS_INFO_STREAM("Reading from serial port\n"); 
            //std_msgs::String result; 
            //result.data = ser.read(ser.available()); 
            //ROS_INFO_STREAM("Read: " << result.data); 
            //read_pub.publish(result); 
        } 
    
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        //ser.write(stop);
        ros::spinOnce();
        loop_rate.sleep(); 
  
    } 
} 
