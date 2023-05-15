//
// Created by zxj on 2023/2/22.
//

#ifndef STANDARD_LIDAR4_WS_DATA_ANAYSIS_H
#define STANDARD_LIDAR4_WS_DATA_ANAYSIS_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <fstream>

namespace anaysis
{


class data_anaysis
{
public:
    data_anaysis()=default;

    data_anaysis(int circles);

    ~data_anaysis(){
        fout_.close();
    }

    double calAveObsolutePresicsion(std::vector<float> &ave_ranges,double real_dist);

    double calRelativePrecision(std::vector<float> &ave_ranges);

    void DataAnaysis(sensor_msgs::LaserScan::ConstPtr scanPtr);



private:
    inline float calMean(std::vector<float> &datas){
        float mean=0;
        for (auto &x:datas){
            mean+=x;
        }
        return mean/datas.size();
    }

    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    std::fstream fout_;

    int circles_;
    int MtoMM_=1000;
    double real_dist_=2.956;


    std::vector<float> high_intens_ave_ranges_;
    std::vector<float> high_intens_ranges_;

};

}

#endif //STANDARD_LIDAR4_WS_DATA_ANAYSIS_H
