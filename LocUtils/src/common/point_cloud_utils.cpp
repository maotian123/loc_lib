//
// Created by Tian on 2023/08/27.
//

#include "LocUtils/common/point_cloud_utils.h"
#include "LocUtils/common/point_types.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

/// 点云的一些工具函数

namespace LocUtils {

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<LocUtils::PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    cloud->swap(*output);
}

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min) {
    CloudPtr output(new PointCloudType);
    for (const auto& pt : cloud->points) {
        if (pt.z > z_min) {
            output->points.emplace_back(pt);
        }
    }

    output->height = 1;
    output->is_dense = false;
    output->width = output->points.size();
    cloud->swap(*output);
}

}  // namespace sad