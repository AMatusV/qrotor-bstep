
#ifndef LPMS_H
#define LPMS_H

#include <ros/ros.h>
#include <ros/package.h> //to get pkg path

//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h> // Added for version 1
#include <controller/FloatList.h>

#include <cstdio>
#include <thread>

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

#include <vector>
#include <deque>

#include <math.h>
const float deg2rad = M_PI/180.0;


typedef struct{

    std::string label;
    int N_data;
    std::vector<double> p;

}my_point;

class lpms_sensor
{
    private:

        ros::NodeHandle nh_;

        int data_size;
        int Max_data;
        std::deque<my_point> data;

        ros::Publisher array_sub;
        std::string mac_addr_device_;
        controller::FloatList angles_msg;
        ros::Subscriber vicon_sub;

        // Gets a LpmsSensorManager instance
        LpmsSensorManagerI* manager;

        LpmsSensorI* lpms;

    public:
        ImuData d;

        ros::Publisher angles_pub;

    public:
        lpms_sensor(ros::NodeHandle nh);
        ~lpms_sensor();
 
        void run(void);


};

#endif
