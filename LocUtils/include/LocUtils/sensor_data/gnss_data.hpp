/*
 * @Date: 2023-10-26 20:54:57
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-29 21:34:55
 * @FilePath: /loc_lib/LocUtils/include/LocUtils/sensor_data/gnss_data.hpp
 */
#pragma once
#include <GeographicLib/LocalCartesian.hpp>

#include <iostream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace LocUtils
{
    class GnssData 
    {
        public:
            GnssData() {}
            ~GnssData() {}
            double timestamp_ = 0.0;
            double latitude_ {0.0};
            double longitude_ {0.0};
            double altitude_ { 0.0 };
            double local_E_ = 0.0;
            double local_N_ = 0.0;
            double local_U_ = 0.0;
            int status_ {0};
            //暂时这些，待补充协方差矩阵，定位状态等。

            static double origin_longitude;
            static double origin_latitude;
            static double origin_altitude;
        private:
            static GeographicLib::LocalCartesian geo_converter;
            static bool origin_position_inited;

        public: 
            void InitOriginPosition();
            void InitManualPosition();
            void UpdateXYZ();
            static void Reverse(
                const double &local_E, const double &local_N, const double &local_U,
                double &lat, double &lon, double &alt
            );

    };
}

using GnssDataPtr = std::shared_ptr<LocUtils::GnssData>;