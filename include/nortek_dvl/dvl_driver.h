/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
*/
#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <serial/serial.h>

#include <nortek_dvl/ButtomTrack.h>
#include <nortek_dvl/CurrentProfile.h>
#include <nortek_dvl/CellMeasure.h>
#include "configure.h"
#include <chrono>
#include <thread>

#define PI 3.1415926
#define BEAM_ANGLE 25.0

class DVLDriver
{

public:
    DVLDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

    ~DVLDriver();

    Result init();

    void readall();

    void publishRaw(const std::string &str);

    void closeSerial();

    // Todos:
    // int read();
    // int write();
    // ...

private:
    serial::Serial *ser;

    bool isOpen = true;

    int sleep_t;

    struct
    {
        std::string frame_id;
        std::string port_name;
        int frequency;
        bool debug;
    }config;
    

    int decode(std::string& stream);

    void decodeBottonTrack(const std::string& str);
    void decodeWaterTrack(const std::string& str);
    void decodeCurrentProfileI(const std::string& str);
    void decodeCurrentProfileS(const std::string& str);
    void decodeCurrentProfileC(const std::string& str);

    bool checkIntegrity(std::string& nmea_data);
    bool calcChecksum(const std::string& nmea_data);

    // ROS
    ros::Publisher bottom_track_pub;
    ros::Publisher currect_profile_pub;
    ros::Publisher raw_pub;
    // specific publish
    ros::Publisher battery_pub;
    ros::Publisher depth_pub;
    ros::Publisher twist_pub;

    ros::Time ioTime;

    nortek_dvl::CurrentProfile cp_msg;


};