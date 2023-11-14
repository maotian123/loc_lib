//
// Created by Tian on 2023/10/31.
//

#include "LocUtils/model/sync/measure_sync.hpp"
namespace LocUtils
{
    MappingMessageSync::MappingMessageSync(MappingSyncOptions option)
        :option_(option)
    {
        imu_sync_ptr_ = std::make_shared<IMUSync>();
    }

    void MappingMessageSync::ProcessCloud(CloudDataFullType &cloud)
    {
        if(cloud.time < last_timestamp_lidar_)
        {
            LOG(ERROR) << "lidar time erro ";
        }

        measures_.Reset();
        measures_.lidar_full_ = cloud.cloud_ptr;
        measures_.sync_time_ = cloud.time;

        last_timestamp_lidar_ = measures_.sync_time_;
    }

    void MappingMessageSync::ProcessCloud(CloudData &cloud)
    {
        if(cloud.time < last_timestamp_lidar_)
        {
            LOG(ERROR) << "lidar time erro ";
        }

        measures_.Reset();
        measures_.lidar_normal_ = cloud.cloud_ptr;
        measures_.sync_time_ = cloud.time;

        last_timestamp_lidar_ = measures_.sync_time_;
    }

    void MappingMessageSync::ProcessImu(std::deque<IMU> &imu_buff)
    {
        if(!option_.b_sync_imu_)
            return;
        double imu_time = imu_buff.front().timestamp_;
        double diff_cloud_imu_time = measures_.sync_time_ - imu_time;

        // LOG(INFO) << "diff_cloud_imu_time " << diff_cloud_imu_time;
        // LOG(INFO) << "measures_.sync_time_ " << measures_.sync_time_;
        if (imu_time > measures_.sync_time_)
        {
            LOG(ERROR) << "imu wait lidar push";
            return;
        }
        while (!imu_buff.empty() && (imu_time < measures_.sync_time_))
        {
            IMU imu_data = imu_buff.front();
            imu_time = imu_data.timestamp_;

            if(imu_time > (measures_.sync_time_ + option_.exced_time_limit_))
            {
                break;
            }

            diff_cloud_imu_time = measures_.sync_time_ - imu_time;
          
            if (diff_cloud_imu_time > option_.behind_time_limit_)
            {
                LOG(INFO) << "cloud time exced too big pop imu " << diff_cloud_imu_time;
                imu_buff.pop_front();
                continue;
            }

            if(imu_time > measures_.sync_time_)
            {
                if(imu_buff.empty())
                {
                    LOG(INFO) << "imu not sync";
                    measures_.b_imu_inter_success_ = false;
                    break;
                }
                IMU sync_imu = ImuSync(measures_.imu_buff_.back(), imu_data);
                measures_.imu_buff_.push_back(sync_imu);
                measures_.b_imu_inter_success_ = true;
                break;
            }
            measures_.imu_buff_.push_back(imu_data);
            imu_buff.pop_front();
        }
        measures_.b_imu_sync_success_ = true;
    }

    void MappingMessageSync::ProcessGnss(std::deque<GnssData> &gnss_buff)
    {

        if(!option_.b_sync_gnss_)
            return;
        double gnss_time = gnss_buff.front().timestamp_;
        if(gnss_time > measures_.sync_time_)
        {
            LOG(ERROR) << "gnss wait lidar push ";
            return;
        }
        while (!gnss_buff.empty() && (gnss_time < measures_.sync_time_))
        {
            gnss_time = gnss_buff.front().timestamp_;
            double diff_cloud_gnss_time = measures_.sync_time_ - gnss_time;
            if(gnss_time > (measures_.sync_time_ + option_.exced_time_limit_))
            {
                break;
            }

            if(diff_cloud_gnss_time > option_.behind_time_limit_)
            {
                LOG(INFO) << "cloud time exced too big pop gnss " << diff_cloud_gnss_time;
                gnss_buff.pop_front();
                continue;
            }

            if(gnss_time > measures_.sync_time_)
            {
                if(gnss_buff.empty())
                {
                    LOG(INFO) << "gnss not sync";
                    measures_.b_gnss_inter_success_ = false;
                    break;
                }
                GnssData sync_gnss = GpsSync(measures_.gnss_buff_.back(), gnss_buff.front());
                measures_.b_gnss_inter_success_ = true;
                measures_.gnss_buff_.push_back(sync_gnss);
                break;
            }
            measures_.gnss_buff_.push_back(gnss_buff.front());
            gnss_buff.pop_front();
        }
        measures_.b_gnss_sync_success_ = true;
    }

    IMU MappingMessageSync::ImuSync(const IMU &front_data, const IMU &back_data)
    {
        IMU synced_data;
        // 匀速 线性插值
        double front_scale = (back_data.timestamp_ - measures_.sync_time_) / (back_data.timestamp_ - front_data.timestamp_);
        double back_scale = (measures_.sync_time_ - front_data.timestamp_) / (back_data.timestamp_ - front_data.timestamp_);
        synced_data.timestamp_ = measures_.sync_time_;

        synced_data.acce_.x() = front_data.acce_.x() * front_scale + back_data.acce_.x() * back_scale;
		synced_data.acce_.y() = front_data.acce_.y() * front_scale + back_data.acce_.y() * back_scale;
		synced_data.acce_.z() = front_data.acce_.z() * front_scale + back_data.acce_.z() * back_scale;

		synced_data.gyro_.x() = front_data.gyro_.x() * front_scale + back_data.gyro_.x() * back_scale;
		synced_data.gyro_.y() = front_data.gyro_.y() * front_scale + back_data.gyro_.y() * back_scale;
		synced_data.gyro_.z() = front_data.gyro_.z() * front_scale + back_data.gyro_.z() * back_scale;

        synced_data.ori.x() = front_data.ori.x() * front_scale + back_data.ori.x() * back_scale;
		synced_data.ori.y() = front_data.ori.y() * front_scale + back_data.ori.y() * back_scale;
		synced_data.ori.z() = front_data.ori.z() * front_scale + back_data.ori.z() * back_scale; 
		synced_data.ori.w() = front_data.ori.w() * front_scale + back_data.ori.w() * back_scale; 

		synced_data.ori.normalize();

		return synced_data;

    }

    GnssData MappingMessageSync::GpsSync(const GnssData &front_data, const GnssData &back_data)
    {
        GnssData synced_data;

        // 匀速 线性插值
        double front_scale = (back_data.timestamp_ - measures_.sync_time_) / (back_data.timestamp_ - front_data.timestamp_);
        double back_scale = (measures_.sync_time_ - front_data.timestamp_) / (back_data.timestamp_ - front_data.timestamp_);
        synced_data.timestamp_ = measures_.sync_time_;
        synced_data.status_ = back_data.status_;
        synced_data.longitude_ = front_data.longitude_ * front_scale + back_data.longitude_ * back_scale;
        synced_data.latitude_ = front_data.latitude_ * front_scale + back_data.latitude_ * back_scale;
        synced_data.altitude_ = front_data.altitude_ * front_scale + back_data.altitude_ * back_scale;

        return synced_data;
    }

    bool MappingMessageSync::IsSync()
    {
        if(option_.b_sync_gnss_)
        {
            if(option_.debug_info_)
            {
                measures_.PrintGnssSyncInfo();
            }
            if(!measures_.b_gnss_sync_success_)
            {
                return false;
            }
        }

        if(option_.b_sync_imu_)
        {
            if(option_.debug_info_)
            {
                measures_.PrintImuSyncInfo();
            }
            if(!measures_.b_imu_sync_success_)
            {
                return false;
            }
        }

        return true;
    }
}

