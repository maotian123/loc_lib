/*
 * @Date: 2023-10-26 21:11:35
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-30 14:36:38
 * @FilePath: /loc_lib/LocUtils/include/LocUtils/common/gnss_utils.hpp
 */

#pragma once
#include <iostream>

#include <LocUtils/sensor_data/gnss_data.hpp>
#include <LocUtils/common/pose_data.hpp>
#include <LocUtils/sensor_data/imu_data.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "eigen_types.h"

#include <gflags/gflags.h>

#include <glog/logging.h>

namespace LocUtils
{
    class GeoToEnu
    {
        public:
            // 默认将第一帧rtk数据当作原点
            GeoToEnu();

            // 可选自定义数据当作原点
            GeoToEnu(double latitude, double longitude, double altitude);
            ~GeoToEnu() {}


            /**
             * @description: 将经纬高转化为ENU东北天坐标系
             * @param {Vec3d&} enu_pose 东北天坐标系位置
             * @param {RtkGpsData} &rtk_data 经纬高数据
             * @return {bool} true：成功
             */
            bool ConvertToENU(Vec3d& enu_pose, GnssData& rtk_data);

            /**
             * @description: 设置初始化标志位
             * @param {bool} value
             * @return {*}
             */        
            void SetInitFlag(bool value);

            /**
             * @description: 获取初始化标志位
             * @return {*}
             */        
            bool GetInitFlag(void);

        private:
            bool init_flag {false}; //初始化标志
            GeographicLib::LocalCartesian geo_converter;
    };

    /**
     * @description: 将gnss位置和imu数据中的姿态合并到PoseData数据结构中(存储时间)
     * @param {Vec3d} point GNSS数据
     * @param {IMU} imu_data IMU数据
     * @param {double} time 时间戳
     * @return {*}
     */    
    inline PoseData MergeToPoseData(Vec3d point, IMU imu_data, double time)
    {
        SE3 pose_buff(imu_data.ori, point);

        PoseData pose_data;

        pose_data.pose_ = LocUtils::SE3TOMAT4(pose_buff).cast<float>();
        pose_data.time_ = time; 
        return pose_data;
    }
    /**
     * @description: 将gnss位置和imu数据中的姿态合并到SE3中(不存储时间)
     * @param {Vec3d} point
     * @param {IMU} imu_data
     * @return {SE3}
     */    
    inline SE3 MergeToSE3(Vec3d point, IMU imu_data)
    {
        SE3 pose_data(imu_data.ori, point);

        return pose_data;
    }

    /**
     * @description: 将gnss位置和imu数据中的姿态合并到Mat4d中(不存储时间)
     * @param {Vec3d} point
     * @param {IMU} imu_data
     * @return {Mat4d}
     */    
    inline Mat4d MergeToMat4d(Vec3d point, IMU imu_data)
    {
        Mat4d pose_data {Eye4d};

        pose_data.block<3, 3>(0, 0) = imu_data.ori.toRotationMatrix();
        pose_data.block<3, 1>(0, 3) = point;

        return pose_data;
    }

    // 时间软同步器
    class GnssSync
    {
        public:
            GnssSync(int min_synv_size, float ege_time_limit);
            ~GnssSync();

            /**
             * @description: 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
             * 即找到与同步时间相邻的左右两个数据
             * 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
             * @param {double} synv_time
             * @return {bool}
             */        
            bool SyncData(std::deque<GnssData>& unsync_datas, std::deque<GnssData>& synced_datas, double sync_time);

            int min_synv_size_;
            float ege_time_limit_;
    };
    // /**
    //  * @description: 按消息队列顺序合并为pose_data, 此函数不会pop_front原来队列的元素, 适用于多线程，可由主线程进行pop
    //  * @param {deque<GnssData>} gnss_data
    //  * @param {deque<IMU>} imu_data
    //  * @param {deque<PoseData>} &pose_data
    //  * @return {*}
    //  */    
    // void MergeTwoDequeToPoseData(std::deque<GnssData> &gnss_data, std::deque<IMU> &imu_data, std::deque<PoseData> &pose_data)
    // {
    //     std::shared_ptr<GeoToEnu> convert_ptr = std::make_shared<GeoToEnu>();
    //     Vec3d gnss_point_in_enu;
    //     PoseData pose_in_enu;

    //     // 直接拷贝，是否会占用大量栈区
    //     // std::deque<GnssData> copy_gnss_data(gnss_data);
    //     // std::deque<IMU> copy_imu_data(imu_data);

    //     // 拷贝队列并放入堆区，防止引用传参影响原队列
    //     std::shared_ptr<std::deque<GnssData>> copy_gnss_data_ptr = std::make_shared<std::deque<GnssData>>(gnss_data);
    //     std::shared_ptr<std::deque<IMU>> copy_imu_data_ptr = std::make_shared<std::deque<IMU>>(imu_data);
        
    //     while(copy_gnss_data_ptr->size() <= copy_imu_data_ptr->size() ? copy_gnss_data_ptr->size() : copy_imu_data_ptr->size())
    //     {
    //         auto temp_gnss_data = copy_gnss_data_ptr->front();
    //         if(false == convert_ptr->ConvertToENU(gnss_point_in_enu, temp_gnss_data))
    //         {
    //             LOG(INFO) << "ConvertToENU failed! KITTI/TUM file may be unavailable.";
    //             continue;
    //         }
    //         auto imu_data_pose = copy_imu_data_ptr->front();

    //         // 简单合并rtk和IMU的pose
    //         pose_in_enu = MergeToPoseData(gnss_point_in_enu, imu_data_pose, temp_gnss_data.time_);//使用RTK时间戳

    //         pose_data.emplace_back(pose_in_enu);
    //         std::cout << pose_data.back().time_ << ":" << std::endl;
    //         std::cout << pose_data.back().pose_ << std::endl; 

    //         copy_gnss_data_ptr->pop_front(); 
    //         copy_imu_data_ptr->pop_front();           
    //     }
    // }
}