//
// Created by Tian on 2023/10/01.
//

#include "LocUtils/model/cloud_filter/no_filter.hpp"

namespace LocUtils 
{
    NoFilter::NoFilter() {
    }

    bool NoFilter::Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) {
        filtered_cloud_ptr.reset(new PointCloudType(*input_cloud_ptr));
        return true;
    }
} 
