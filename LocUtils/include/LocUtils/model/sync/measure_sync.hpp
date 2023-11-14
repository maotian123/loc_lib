//
// Created by Tian on 2023/10/31.
//

#pragma once

#include "LocUtils/common/point_types.h"
#include "LocUtils/sensor_data/imu_data.hpp"
#include "LocUtils/sensor_data/gnss_data.hpp"
#include "LocUtils/sensor_data/cloud_data.hpp"

#include <memory>
#include <glog/logging.h>

/*
    NOTE 
    1、只在mapping的情况下进行sync
    2、必须先使用 process cloud 
        在根据 输入 imu 和 gnss
*/

namespace LocUtils
{
    struct MappingSyncOptions
    {
        // 最多超过的值,做插值
        double exced_time_limit_ {0.02};
        // 雷达数据之前的最多时间
        double behind_time_limit_ {0.2};
        
        bool b_sync_imu_ {true};
        bool b_sync_gnss_ {true};
        bool debug_info_ {false};
    };

    struct MappingMeasureGroup 
    {
        MappingMeasureGroup()
            :lidar_full_ (new FullPointCloudType())
            ,lidar_normal_ (new PointCloudType())
        {      
        };

        void Reset()
        {
            imu_buff_.clear();
            gnss_buff_.clear();
            lidar_full_.reset(new FullPointCloudType());
            lidar_normal_.reset(new PointCloudType());
            b_imu_sync_success_ = false;
            b_gnss_sync_success_ = false;
            b_imu_inter_success_ = false;
            b_gnss_inter_success_ = false;
        }

        void PrintImuSyncInfo()
        {
            LOG(INFO) << " ------- print imu sync data -------\n"
                      << " imu sync success " << b_imu_sync_success_ << "\n"
                      << " has imu inter data " << b_imu_inter_success_;
            LOG(INFO) << "imu lidar diff time: \n";
            for (int i{0}; i < imu_buff_.size(); ++i)
            {
                double diff_cloud_imu_time  = sync_time_ - imu_buff_.at(i).timestamp_;
                std::cout << " imu data idx " << i << " : " << diff_cloud_imu_time << "\n";
            }
        }
        
        void PrintGnssSyncInfo()
        {
            LOG(INFO) << " ------- print gnss sync data -------\n"
                      << " gnss sync success " << b_gnss_sync_success_ << "\n"
                      << " has gnss inter data " << b_gnss_inter_success_;
            LOG(INFO) << "gnss lidar diff time: \n";
            for (int i{0}; i < gnss_buff_.size(); ++i)
            {
                double diff_cloud_gnss_time  = sync_time_ - gnss_buff_.at(i).timestamp_;
                std::cout << " gnss data idx " << i << " : " << diff_cloud_gnss_time << "\n";
            }
        }

        double sync_time_ = 0;   // 雷达包的起始时间
        FullCloudPtr lidar_full_ {nullptr};  // 雷达点云
        CloudPtr lidar_normal_ {nullptr};  // 雷达点云

        bool b_imu_sync_success_ {false};
        bool b_gnss_sync_success_ {false};

        bool b_imu_inter_success_ {false};
        bool b_gnss_inter_success_ {false};
        std::deque<IMU> imu_buff_;
        std::deque<GnssData> gnss_buff_;
    };

    class MappingMessageSync 
    {
        public:
            MappingMessageSync(MappingSyncOptions option);
            
            void ProcessCloud(CloudDataFullType &cloud);
            void ProcessCloud(CloudData &cloud);
            void ProcessImu(std::deque<IMU> &imu_buff);
            void ProcessGnss(std::deque<GnssData> &gnss_buff);

            bool IsSync();

            MappingMeasureGroup GetMeasure()
            {
                return measures_;
            }
        private:
            /// 同步，成功时返回true
            IMU ImuSync(const IMU &front_data, const IMU &back_data);
            GnssData GpsSync(const GnssData &front_data, const GnssData &back_data);

        private:
            std::shared_ptr<IMUSync> imu_sync_ptr_;
            MappingMeasureGroup measures_;

            MappingSyncOptions option_;
            
            double last_timestamp_imu_ = -1.0;              // 最近imu时间
            double last_timestamp_lidar_ = 0;               // 最近lidar时间
    };
}