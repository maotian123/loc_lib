//
// Created by Tian on 2023/10/01.
//

#pragma once
#include "LocUtils/common/point_types.h"

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
inline void SaveCloudToFile(const std::string &filePath, CloudPtr &cloud) {
    cloud->height = 1;
    cloud->width = cloud->size();
    pcl::io::savePCDFileASCII(filePath, *cloud);
}

//读取点云PCD文件(就一句话，还是封装1下吧)
inline void ReadCloudFromPCDFile(const std::string& filePath, CloudPtr& cloud)
{
    pcl::io::loadPCDFile(filePath, *cloud);
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

