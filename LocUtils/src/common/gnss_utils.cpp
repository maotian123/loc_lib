/*
 * @Date: 2023-10-26 21:39:09
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-30 19:10:35
 * @FilePath: /loc_lib/LocUtils/src/common/gnss_utils.cpp
 */
#include <LocUtils/common/gnss_utils.hpp>

namespace LocUtils
{
    GeoToEnu::GeoToEnu()
    {
        init_flag = false;
    }

    GeoToEnu::GeoToEnu(double latitude, double longitude, double altitude)
    {
        geo_converter.Reset(latitude, longitude, altitude);
        init_flag = true;
    }

    bool GeoToEnu::ConvertToENU(Vec3d& enu_pose, GnssData& rtk_data)
    {
        if(!init_flag)
        {
            geo_converter.Reset(rtk_data.latitude_, rtk_data.longitude_, rtk_data.altitude_);
            init_flag = true;
            // std::cout << "init = true" << std::endl;
            enu_pose.x() = 0.0;
            enu_pose.y() = 0.0;
            enu_pose.z() = 0.0;
            return true;
        }

        geo_converter.Forward(rtk_data.latitude_, rtk_data.longitude_, rtk_data.altitude_, \
            enu_pose.x(), enu_pose.y(), enu_pose.z());

        return true;
    }

    void GeoToEnu::SetInitFlag(bool value)
    {
        init_flag = value;
    }

    bool GeoToEnu::GetInitFlag(void)
    {
        return init_flag;
    }

    GnssSync::GnssSync(int min_synv_size, float ege_time_limit)
    :min_synv_size_(min_synv_size),ege_time_limit_(ege_time_limit)
    {}

    GnssSync::~GnssSync(){}

    bool GnssSync::SyncData(std::deque<GnssData>& unsync_datas, std::deque<GnssData>& synced_datas, double sync_time)
    {
        while(unsync_datas.size() >= min_synv_size_)
        {
            if(unsync_datas.front().timestamp_ > sync_time) 
            {
                return false;
            }
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

        GnssData front_data = unsync_datas.at(0);
        GnssData back_data = unsync_datas.at(1);
        GnssData synced_data;

        // 匀速 线性插值
        double front_scale = (back_data.timestamp_ - sync_time) / (back_data.timestamp_ - front_data.timestamp_);
        double back_scale = (sync_time - front_data.timestamp_) / (back_data.timestamp_ - front_data.timestamp_);
        synced_data.timestamp_ = sync_time;
        synced_data.status_ = back_data.status_;
        synced_data.longitude_ = front_data.longitude_ * front_scale + back_data.longitude_ * back_scale;
        synced_data.latitude_ = front_data.latitude_ * front_scale + back_data.latitude_ * back_scale;
        synced_data.altitude_ = front_data.altitude_ * front_scale + back_data.altitude_ * back_scale;
        // synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        // synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        // synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

        synced_datas.emplace_back(synced_data);

        return true;
    }
}