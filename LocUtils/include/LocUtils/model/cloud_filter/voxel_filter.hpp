//
// Created by Tian on 2023/09/29.
//

#pragma once
#include "LocUtils/model/cloud_filter/cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace LocUtils
{
    class VoxelFilter: public CloudFilterInterface
    {
        public:
            VoxelFilter(float voxel_size = 0.5);
            bool Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) override;
        private:
            pcl::VoxelGrid<PointType> voxel_filter_;

    };
}

