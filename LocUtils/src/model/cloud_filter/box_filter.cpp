//
// Created by Tian on 2023/10/31.
//

// #include "LocU"

#include "LocUtils/model/cloud_filter/box_filter.hpp"
#include <vector>
#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace LocUtils
{
    BoxFilter::BoxFilter(float step_x, float step_y, float step_z)
    { 
        size_.resize(6);
        edge_.resize(6);
        origin_.resize(3);

        size_ = {-step_x, step_x, -step_y, step_y, -step_z, step_z};
        SetSize(size_);
    }

    bool BoxFilter::Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr)
    {
        filtered_cloud_ptr->clear();
        pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
        pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
        pcl_box_filter_.setInputCloud(input_cloud_ptr);
        pcl_box_filter_.filter(*filtered_cloud_ptr);
    }

    void BoxFilter::SetSize(std::vector<float> size)
    {
        size_ = size;
        LOG(INFO) << "Box Filter params:" << "\n"
                  << "min_x: " << size.at(0) << ", "
                  << "max_x: " << size.at(1) << ", "
                  << "min_y: " << size.at(2) << ", "
                  << "max_y: " << size.at(3) << ", "
                  << "min_z: " << size.at(4) << ", "
                  << "max_z: " << size.at(5);
        CalculateEdge();
    }

    void BoxFilter::SetOrigin(std::vector<float> origin) 
    {
        origin_ = origin;
        CalculateEdge();
    }
    
    void BoxFilter::CalculateEdge() 
    {
        for (size_t i = 0; i < origin_.size(); ++i) 
        {
            edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
            edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
        }
    }

    std::vector<float> BoxFilter::GetEdge() 
    {
        return edge_;
    }
}