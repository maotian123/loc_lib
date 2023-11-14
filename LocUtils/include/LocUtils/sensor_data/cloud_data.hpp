//
// Created by Tian on 2023/09/05.
//

#ifndef LOCUTILS_SENSOR_DATA_CLOUD_DATA_HPP_
#define LOCUTILS_SENSOR_DATA_CLOUD_DATA_HPP_

#include "LocUtils/common/point_types.h"

namespace LocUtils 
{
template<typename PointT>
class CloudDataTemplate {
  public:
  public:
    CloudDataTemplate()
      :cloud_ptr(new typename pcl::PointCloud<PointT>) {
    }

  public:
    double time = 0.0;
    typename pcl::PointCloud<PointT>::Ptr cloud_ptr;
};

using CloudData = CloudDataTemplate<PointType>;
using CloudDataFullType = CloudDataTemplate<FullPointType>;
using CloudDataRsLidar = CloudDataTemplate<::rslidar_ros::Point>;
using CloudDataVelodyneLidar = CloudDataTemplate<::velodyne_ros::Point>;

}

#endif