//
// Created by Tian on 2023/10/01.
//

#pragma once
#include "LocUtils/common/point_types.h"

#include <glog/logging.h>

/// 点云的一些工具函数

namespace LocUtils {


inline CloudPtr RemoveNanPoint(const CloudPtr& input)
{
    std::vector<int> indices;
    CloudPtr output(new PointCloudType);
    pcl::removeNaNFromPointCloud(*input, *output, indices);

    return output;
}

// 保存点云文件
inline bool SaveCloudToFile(const std::string &filePath, CloudPtr &cloud) {
    cloud->height = 1;
    cloud->width = cloud->size();
    int ret = pcl::io::savePCDFileASCII(filePath, *cloud);
    if (ret != 0) {
        LOG(ERROR) << "Failed to save PCD file: " << filePath;
        return false;
    }
    return true;
}

//读取点云PCD文件
inline bool ReadCloudFromPCDFile(const std::string& filePath, CloudPtr& cloud)
{
    int ret = pcl::io::loadPCDFile(filePath, *cloud);
    if (ret != 0) {
        LOG(ERROR) << "Failed to load PCD file: " << filePath;
        return false;
    }
    return true;
}
/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

template<typename Point>
float PointDistance(Point p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

}  

