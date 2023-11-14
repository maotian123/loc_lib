//
// Created by Tian on 2023/10/01.
//

#include "LocUtils/model/cloud_filter/voxel_filter.hpp"
#include <glog/logging.h>

namespace LocUtils
{
    VoxelFilter::VoxelFilter(float voxel_size)
    {
        voxel_filter_.setLeafSize(voxel_size, voxel_size, voxel_size);

        LOG(INFO) << "Voxel Filter params:" << "\n"
              << voxel_size 
              << "\n" << "\n";
    }

    bool VoxelFilter::Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) 
    {
        voxel_filter_.setInputCloud(input_cloud_ptr);

        voxel_filter_.filter(*filtered_cloud_ptr);
        return true;
    }
}
