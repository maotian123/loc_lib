/*
 * @Date: 2023-10-29 21:19:58
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-30 19:10:17
 * @FilePath: /loc_lib/LocUtils/src/common/imu.cpp
 */
#include <LocUtils/sensor_data/imu_data.hpp>

namespace LocUtils
{
	IMUSync::IMUSync(int min_synv_size, float ege_time_limit)
    :min_synv_size_(min_synv_size),ege_time_limit_(ege_time_limit)
    {}

    IMUSync::~IMUSync()
    {}

    bool IMUSync::SyncData(std::deque<IMU>& unsync_datas, std::deque<IMU>& synced_datas, double sync_time)
    {
        synced_datas.clear();
        while (unsync_datas.size() >= min_synv_size_)
        {
            if(unsync_datas.front().timestamp_ > sync_time) return false;
            if(unsync_datas.at(1).timestamp_ < sync_time)
            {
                unsync_datas.pop_front();
                continue;
            }

            if(sync_time - unsync_datas.front().timestamp_ > ege_time_limit_)
            {
                unsync_datas.pop_front();
                return false;
            }
            if(unsync_datas.at(1).timestamp_ - sync_time > ege_time_limit_)
            {
                unsync_datas.pop_front();
                return false;
            }
            break;
        }

        if(unsync_datas.size() < min_synv_size_)
            return false;

        IMU front_data = unsync_datas.at(0);
        IMU back_data = unsync_datas.at(1);
        IMU synced_data;

        // 匀速 线性插值
        double front_scale = (back_data.timestamp_ - sync_time) / (back_data.timestamp_ - front_data.timestamp_);
        double back_scale = (sync_time - front_data.timestamp_) / (back_data.timestamp_ - front_data.timestamp_);
        synced_data.timestamp_ = sync_time;

        synced_data.acce_.x() = front_data.acce_.x() * front_scale + back_data.acce_.x() * back_scale;
		synced_data.acce_.y() = front_data.acce_.y() * front_scale + back_data.acce_.y() * back_scale;
		synced_data.acce_.z() = front_data.acce_.z() * front_scale + back_data.acce_.z() * back_scale;

		synced_data.gyro_.x() = front_data.gyro_.x() * front_scale + back_data.gyro_.x() * back_scale;
		synced_data.gyro_.y() = front_data.gyro_.y() * front_scale + back_data.gyro_.y() * back_scale;
		synced_data.gyro_.z() = front_data.gyro_.z() * front_scale + back_data.gyro_.z() * back_scale;

		// 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
		synced_data.ori.x() = front_data.ori.x() * front_scale + back_data.ori.x() * back_scale;
		synced_data.ori.y() = front_data.ori.y() * front_scale + back_data.ori.y() * back_scale;
		synced_data.ori.z() = front_data.ori.z() * front_scale + back_data.ori.z() * back_scale; 
		synced_data.ori.w() = front_data.ori.w() * front_scale + back_data.ori.w() * back_scale; 

		synced_data.ori.normalize();

		synced_datas.emplace_back(synced_data);

		return true;
    }	

}