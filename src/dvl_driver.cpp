/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
* For NMEA message structure, look here: https://en.wikipedia.org/wiki/NMEA_0183
*/

#include <nortek_dvl/dvl_driver.h>
#include <iostream>
#include <string>
#include <nortek_dvl/tic_toc.h>
#include <unistd.h>

DVLDriver::DVLDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    // before open it, check if it still open? 

    // check param
    private_nh.param("frame_id", config.frame_id, std::string(FRAME_ID));
    private_nh.param("port_name", config.port_name, std::string(DEFAULT_PORT));

    private_nh.param("debug", config.debug, DEBUG);
    
    // private_nh.param("frequency", config.frequency, DEFAULT_FREQUENCY);
    // sleep_t = round(config.frequency/63.0*1000000) + 450 + 2000;


    // publisher
    bottom_track_pub= node.advertise<nortek_dvl::ButtomTrack>("/rov/sensors/dvl/buttom_track",10);
    currect_profile_pub= node.advertise<nortek_dvl::CurrentProfile>("/rov/sensors/dvl/current_profile",5);
    raw_pub = node.advertise<std_msgs::String>("/rov/sensors/dvl/raw",5);
    battery_pub = node.advertise<std_msgs::Float32>("/rov/sensors/dvl/battery",1);
    depth_pub = node.advertise<std_msgs::Float32>("/rov/sensors/dvl/depth",1);
    twist_pub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("/rov/sensors/dvl/twist",5);
}

DVLDriver::~DVLDriver()
{
    // serial bug, if port not open, use close() will have a Segmentation fault
    if(isOpen)
        ser->close();

    delete[] ser;

}

Result DVLDriver::init()
{
    Result Err = CORRECT;
    try
    {
        ser = new serial::Serial(config.port_name,DEFAULT_BAUD,serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
        // ser = new serial::Serial(config.port_name,1250000,serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));

        //clear the buffer for new data
        ser->flushInput();
    }
    catch (serial::IOException& e)
    {
        Err = ERR_SERIAL;

        isOpen = false;
    }    

    return Err;
}

void DVLDriver::closeSerial()
{
    if(isOpen)
        ser->close();

    delete[] ser;

    ROS_INFO("DVL Driver : serial closed");    
}

void DVLDriver::readall()
{
    std::string data;
    std::size_t sz_t = 120;
    std::string eol;
    eol.append(1,0xd);
    eol.append(1,0xa);
    
    ser->flushInput();

    while(1)
    {
        data.clear();

        // the parse process take about ? millis (for 63 lines)
        if(ser->available()){

            // TicToc t;

            data = ser->readline(sz_t,eol);
            ioTime = ros::Time::now();

            if(config.debug)
                publishRaw(data);

            decode(data);  

            // printf(" proc t:%f\n", t.toc());

        }

        std::chrono::microseconds dura(1);
        std::this_thread::sleep_for(dura);

    }
}

/*
*For test
*/
void DVLDriver::publishRaw(const std::string &str)
{
    // ROS_INFO("DVL Deubg: data=%s", data.c_str());
    std_msgs::String raw;
    raw.data = str;

    raw_pub.publish(raw);
}


int DVLDriver::decode( std::string& stream )
{
    int size = stream.size();

    // check integrity(first read):  $,data,00<CR><LF> 
    /* delete bad characters, like: null(ASCII:0)*/
    if(!checkIntegrity(stream)){
        
        return -1;
    }

    // check checksum
    bool checked = calcChecksum(stream);

    // check data format
    char id;
    if (checked){
        id = stream.at(5);
    }
    else{
        ROS_INFO("DVL Driver warning: bad checksum");
        return -1;
    }

    switch(id)
    {
        // Botton track
        case BOTTON_TRACK:
            decodeBottonTrack(stream);
            break;
        // water track
        case WATER_TRACK:
            decodeWaterTrack(stream);
            break;
        // Curret profile
        case CURRENT_PROFILE_I:
            decodeCurrentProfileI(stream);
            break;
        case CURRENT_PROFILE_S:
            decodeCurrentProfileS(stream);
            break;
        case CURRENT_PROFILE_C:
            decodeCurrentProfileC(stream);
            break;

        default:
            ROS_INFO("DVL Driver warning: bad format id");
            return -1;
    }

    return 0;
}

void DVLDriver::decodeBottonTrack(const std::string& str)
{
    /*size: 100+ char*/

    nortek_dvl::ButtomTrack msg;

    char time[20];

    const char *c = str.c_str();

    if(sscanf(c, "$PNORBT7,%15s,%f,%f,%lf,%lf,%lf,%f,%f,%f,%f,%f", 
                   time, &msg.dt_1, &msg.dt_2, 
                   &msg.speed.x, &msg.speed.y, &msg.speed.z, &msg.figure_of_merit, 
                   &msg.vertical_distance[0], &msg.vertical_distance[1], &msg.vertical_distance[2], &msg.vertical_distance[3]) >= 1) 
    {
        // // TEST:
        // printf("BT SYS: %s;  ",time);
        // printf("ROS: %f\n",ioTime.toSec());
        
        
        // publish buttom trak
        msg.header.stamp = ioTime;
        msg.header.frame_id = config.frame_id;
        bottom_track_pub.publish(msg);

        // publish depth from buttom track
        std_msgs::Float32 depthMsg;
        depthMsg.data = (msg.vertical_distance[0]*cos(BEAM_ANGLE*PI/180.0) +
                      msg.vertical_distance[1]*cos(BEAM_ANGLE*PI/180.0) +
                      msg.vertical_distance[2]*cos(BEAM_ANGLE*PI/180.0) +
                      msg.vertical_distance[3]*cos(BEAM_ANGLE*PI/180.0)) / 4;
        depth_pub.publish(depthMsg);

        // publish velocity from buttom track
        geometry_msgs::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.stamp = ioTime;
        twistMsg.header.frame_id = config.frame_id;
        twistMsg.twist.twist.linear.x = msg.speed.x;
        twistMsg.twist.twist.linear.y = msg.speed.y;
        twistMsg.twist.twist.linear.z = msg.speed.z;
        twistMsg.twist.covariance[0] = 0.005*0.005;
        twistMsg.twist.covariance[7] = 0.005*0.005;
        twistMsg.twist.covariance[14] = 0.005*0.005;
        twist_pub.publish(twistMsg);
    }
    
}

void DVLDriver::decodeWaterTrack(const std::string& str)
{
    // may be in the future
}

void DVLDriver::decodeCurrentProfileI(const std::string& str)
{
    /*size: 40+ char*/


    char coordinate_system[4];

    const char *c = str.c_str();
    if(sscanf(c, "$PNORI1,%hhu,%d,%hhu,%hhu,%f,%f,%4s", 
                   &cp_msg.instrument_type, &cp_msg.header.seq, &cp_msg.beam_number, &cp_msg.cell_number,
                   &cp_msg.blanking_distance, &cp_msg.cell_size, coordinate_system) >= 1) 
    {
        cp_msg.header.frame_id = config.frame_id;

        cp_msg.coordinate_system = coordinate_system; // XYZ*
    }
}

void DVLDriver::decodeCurrentProfileS(const std::string& str)
{
    /*size: 90+ char*/

    int date;
    int time;

    const char *c = str.c_str();

    if(sscanf(c, "$PNORS1,%d,%d,%hhd,%d,%f,%f,%lf,%lf,%lf,%lf,%lf,%lf,%f,%f,%f", 
                   &date, &time, &cp_msg.error, &cp_msg.status, &cp_msg.battery, &cp_msg.sound_speed, 
                   &cp_msg.atitude.z, &cp_msg.atitude_std_dev.z, &cp_msg.atitude.y, &cp_msg.atitude_std_dev.y, 
                   &cp_msg.atitude.x, &cp_msg.atitude_std_dev.x, &cp_msg.pressure, &cp_msg.pressure_std_dev, &cp_msg.temperature) >= 1) 
    {

        cp_msg.header.stamp = ioTime; 

        std_msgs::Float32 battery;
        battery.data = cp_msg.battery;
        battery_pub.publish(battery);
    }
}

void DVLDriver::decodeCurrentProfileC(const std::string& str)
{
    /*size: 90+ char*/


    nortek_dvl::CellMeasure cell;

    const char *c = str.c_str();

    int date, time; // DVL cell time not using here, just using ROS time

    if(sscanf(c, "$PNORC1,%d,%d,%hu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%hu,%hu,%hu,%hu", 
                   &date, &time, &cell.cell_num, &cell.pos, 
                   &cell.v_x,   &cell.v_y,   &cell.v_z,   &cell.v_w, 
                   &cell.amp_x, &cell.amp_y, &cell.amp_z, &cell.amp_w, 
                   &cell.cor_x, &cell.cor_y, &cell.cor_z, &cell.cor_w) >= 1) 
    {
        // // TEST:
        // printf("C no.%d, SYS: %d,%d;  ",cell.cell_num, date, time);
        // printf("ROS: %f,",ioTime.toSec());

        cell.stamp = ioTime;
        cp_msg.cells.push_back(cell);

        // full currect profile is received as followed: 1 PNORI1, 1 PNORS1, 60 PNORC1
        // TODO: realtime determine this cell sizes by setting

       
        if(cell.cell_num==cp_msg.cell_number){
            currect_profile_pub.publish(cp_msg);
            cp_msg.cells.clear();
        }
    }

}



bool DVLDriver::checkIntegrity(std::string& nmea_data)
{
    // return error_value

    int sz = nmea_data.size();
    int i;

    // check min size
    if(sz<=6){
        ROS_INFO("DVL Driver Warning: less than min size");
        return false;
    }

    // find bad char
    bool isFound = false;
    std::vector<int> bad_char;
    for(i=0; i<sz; i++)
    {
        if( (nmea_data[i] < 32 || nmea_data[i] > 126) && nmea_data[i] != 10 && nmea_data[i] != 13){//ASCII: "Space" and "~"
            bad_char.push_back(i);
            isFound = true;
        } 
    }
    // delete bad char
    if(isFound){
        for(i=0; i<bad_char.size(); i++)
        {
            nmea_data.erase(bad_char[i]-i,1);
        }

        ROS_INFO("DVL Driver Warning: bad char sz is: %ld\n", bad_char.size());
    }

    //$,data,...,data*00<CR><LR>
    //Check: Start delimiter, Checksum delimiter, 2 bits checksum, Carriage return, Line feed
    sz = nmea_data.size();
    if(!(nmea_data.at(0)=='$' && nmea_data.at(sz - 5)=='*')){
        ROS_INFO("DVL Driver Warning: bad integrality");
        return false;
    }

    return true;
}

bool DVLDriver::calcChecksum(const std::string& nmea_data)
{
    //NMNA0183
    int sz = nmea_data.size();
    int i, crc=0;

    //$*00<CR><LF>
    for (i = 1; i < sz - 5; i ++) {
        crc ^= nmea_data[i];
    }
    
    // extract 2 bits checksum
    int checksum = std::stoi(nmea_data.substr(sz-4, 2), 0, 16);

    return crc == checksum;
}