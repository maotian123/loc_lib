/*
 * @Author: wei-zhifei afeii2@126.com
 * @Date: 2023-10-28 19:05:39
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-29 21:22:45
 * @FilePath: /loc_lib/LocUtils/include/LocUtils/common/imu.h
 */
#pragma once

#include <iostream>
#include <deque>
#include <memory>
#include "LocUtils/common/eigen_types.h"

namespace LocUtils {

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero(); //x y z
    Vec3d acce_ = Vec3d::Zero(); //x y z
    Quatd ori = Quatd::Identity();
};


    // 时间软同步器
class IMUSync
{
    public:
        IMUSync(int min_synv_size = 2, float ege_time_limit = 0.2f);
        ~IMUSync();

        /**
         * @description: 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
         * 即找到与同步时间相邻的左右两个数据
         * 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
         * @param {double} sync_time
         * @return {bool}
         */        
        bool SyncData(std::deque<IMU>& unsync_datas, std::deque<IMU>& synced_datas, double sync_time);

        int min_synv_size_;
        float ege_time_limit_;
};

}  

using IMUPtr = std::shared_ptr<LocUtils::IMU>;

