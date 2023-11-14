//
// Created by Tian on 2023/11/05.
//

#include "LocUtils/sensor_data/gnss_data.hpp"

//静态成员变量必须在类外初始化
double LocUtils::GnssData::origin_latitude = 0.0;
double LocUtils::GnssData::origin_longitude = 0.0;
double LocUtils::GnssData::origin_altitude = 0.0;
bool LocUtils::GnssData::origin_position_inited = false;
GeographicLib::LocalCartesian LocUtils::GnssData::geo_converter;

namespace LocUtils
{
    void GnssData::InitOriginPosition() 
    {
        geo_converter.Reset(latitude_, longitude_, altitude_);

        origin_latitude = latitude_;
        origin_longitude = longitude_;
        origin_altitude = altitude_;

        origin_position_inited = true;
    }

    void GnssData::InitManualPosition() 
    {
        geo_converter.Reset(origin_latitude, origin_longitude, origin_altitude);

        origin_position_inited = true;
    }

    void GnssData::UpdateXYZ() {
        
        if (!origin_position_inited) {
            LOG(ERROR) << "WARNING: GeoConverter is NOT initialized.";
        }
        // LOG(INFO) << origin_latitude;
        geo_converter.Forward(latitude_, longitude_, altitude_, local_E_, local_N_, local_U_);
    }

    void GnssData::Reverse(
        const double &local_E, const double &local_N, const double &local_U,
        double &lat, double &lon, double &alt
    ) {
        if (!origin_position_inited) {
            LOG(ERROR) << "WARNING: GeoConverter is NOT initialized.";
        }

        geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
    }

}