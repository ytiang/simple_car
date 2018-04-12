//
// Created by yangt on 1/31/18.
//
#ifndef SIMPLE_CAR_SIMPLE_CAR_NODE_HPP
#define SIMPLE_CAR_SIMPLE_CAR_NODE_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
//#include <internal_grid_map/internal_grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>


#define GO_STRIGHT 1


#define TURN_RIGHT 4
#define TURN_LEFT 2

#define  STOP 1
#define GO 2
#define BACK 4

#define DIS_THRESHOLD 5

typedef struct Block{
    double v;
    int begin_id;
    int end_id;
}Block;

typedef struct DriveMode{
    uint8_t steering;
    uint8_t left_mode;
    uint8_t right_mode;
}driveMode;

class SimpleCar{
 public:
    SimpleCar();
    ~SimpleCar();
    bool setSerial(std::string port = "/dev/ttyUSB1",
                   uint32_t baundRate = 9600);
    void sendCommand(const driveMode &cmd);
 private:
    grid_map::GridMap gridmap_;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer timer_;
    ros::Publisher map_pub_;
    ros::Publisher poly_pub_;
    serial::Serial my_serial_;
    ros::NodeHandle nh_;
    DriveMode drive_mode_;
    DriveMode drive_mode_old_;
    int back_count_;
    int go_count_;
    bool enable_auto_drive_;
    sensor_msgs::LaserScan laser_msg_;

    void calSteering();
    bool updateGridMap();
    /// callback function
    void laserScanCb(const sensor_msgs::LaserScan &laser_msgs);
    void timerCb();
    void keyCommandCb(const geometry_msgs::Twist &vel_msgs);
};

#endif //SIMPLE_CAR_SIMPLE_CAR_NODE_HPP
