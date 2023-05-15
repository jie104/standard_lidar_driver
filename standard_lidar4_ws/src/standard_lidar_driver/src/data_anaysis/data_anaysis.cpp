//
// Created by zxj on 2023/2/23.
//
#include "data_anaysis.h"
#include <cmath>
namespace anaysis{

    data_anaysis::data_anaysis(int circles):circles_(circles){
        fout_.open("/home/zxj/data/data_anaysis/data_anaysis.txt",std::ios::out);
        sub_=nh_.subscribe("oradar_node/scan",10,&data_anaysis::DataAnaysis,this);
    }

    double data_anaysis::calAveObsolutePresicsion(std::vector<float> &ave_ranges,double real_dist){
        int size=ave_ranges.size();
        double sum=0;
        for (auto &range:ave_ranges){
            double obsolute_range= fabs(range-real_dist);
            sum+=obsolute_range;
        }
        double ave_obsolute_precision=sum/size*MtoMM_;
        fout_ << "average obsolute precision: " << ave_obsolute_precision << " mm" << std::endl;
        std::cout << "average obsolute precision: " << ave_obsolute_precision << " mm" << std::endl;
        return ave_obsolute_precision;
    }

    double data_anaysis::calRelativePrecision(std::vector<float> &ave_ranges){  //3*sigma
        auto mean= calMean(ave_ranges);
        double sum=0;
        int size=ave_ranges.size();
        for (auto &range:ave_ranges){
            sum+=(range-mean)*(range-mean);
        }
        double ave_relative_precision=3*std::sqrt(sum/size)*MtoMM_;
        fout_ << " average relative precision(3*sigma): " << ave_relative_precision << " mm" << std::endl;
        std::cout << " average relative precision(3*sigma): " << ave_relative_precision << " mm" << std::endl;
        return ave_relative_precision;
    }

     void data_anaysis::DataAnaysis(sensor_msgs::LaserScan::ConstPtr scanPtr){
        std::vector<float> ranges= scanPtr->ranges;
        std::vector<float> intens=scanPtr->intensities;
//        std::cout << "22222" << std::endl;
        int size=ranges.size();
        int index=0;
        for (int i=0;i<size;i++){
            index++;
            if (index >700 && index <750){
//                std::cout << "3333" << std::endl;
                if (intens[index] >500){
                    auto range=ranges[index];
                    high_intens_ranges_.push_back(range);
                }
            }
        }
        high_intens_ave_ranges_.push_back(calMean(high_intens_ranges_));
//        std::cout << "size: " << high_intens_ranges_.size()  << std::endl;
        high_intens_ranges_.clear();

        int high_intens_ave_ranges_size=high_intens_ave_ranges_.size();
        if(high_intens_ave_ranges_size > circles_){
            calAveObsolutePresicsion(high_intens_ave_ranges_,real_dist_);
            calRelativePrecision(high_intens_ave_ranges_);
//            std::cout << "high_intens_ave_ranges_size： " << high_intens_ave_ranges_size << std::endl;
            high_intens_ave_ranges_.clear();
        }

    }


}

