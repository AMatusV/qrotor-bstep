

#ifndef LPMS_H
#define LPMS_H

#include <ros/ros.h>
#include <ros/package.h> //to get pkg path


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>

#include <cstdio>
#include <thread>

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

#include <vector>
#include <deque>


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

        // Gets a LpmsSensorManager instance
        LpmsSensorManagerI* manager;

        LpmsSensorI* lpms;

    public:
        ImuData d;

        ros::Publisher pub_acc;

    public:
        lpms_sensor(ros::NodeHandle nh);
        ~lpms_sensor();


        void run(void);


};

#endif
