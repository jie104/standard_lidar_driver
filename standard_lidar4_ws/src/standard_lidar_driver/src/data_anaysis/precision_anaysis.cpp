//
// Created by zxj on 2023/2/22.
//

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "data_anaysis.h"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"precision_anaysis");
    anaysis::data_anaysis ORADAR_DATA_ANAYSIS(20);

    ros::spin();
}