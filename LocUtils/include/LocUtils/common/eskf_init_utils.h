//
// Shared ESKF / IMU initialization utilities used by Lio and Loc.
//
#pragma once

#include <deque>
#include <memory>

#include <glog/logging.h>

#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/math_utils.h"
#include "LocUtils/model/static_init/static_imu_init.h"
#include "LocUtils/model/eskf/eskf.hpp"
#include "LocUtils/sensor_data/imu_data.hpp"

namespace LocUtils {

inline SE3 InitEskfCommon(std::shared_ptr<StaticIMUInit>& static_imu_init_ptr,
                          std::shared_ptr<ESKFD>& eskf_ptr,
                          const std::vector<double>& ext_t,
                          const std::vector<double>& ext_r)
{
    static_imu_init_ptr = std::make_shared<StaticIMUInit>();
    eskf_ptr = std::make_shared<ESKFD>();

    Vec3d T_imu_lidar = math::VecFromArray(ext_t);
    Mat3d R_imu_lidar = math::RpyToRotM2(ext_r[0], ext_r[1], ext_r[2]);

    SE3 TIL(R_imu_lidar, T_imu_lidar);
    LOG(INFO) << "---------- TIL_ ---------" << TIL.matrix();
    return TIL;
}

inline bool InitImuCommon(std::shared_ptr<StaticIMUInit>& static_imu_init_ptr,
                          std::shared_ptr<ESKFD>& eskf_ptr,
                          std::deque<IMU>& imu_buff,
                          bool& has_init_eskf)
{
    if (static_imu_init_ptr == nullptr)
        static_imu_init_ptr = std::make_shared<StaticIMUInit>();

    if (static_imu_init_ptr->InitSuccess())
    {
        LOG(INFO) << " static imu init ok ";
        return true;
    }

    while (!imu_buff.empty())
    {
        IMU cur_imu_data = imu_buff.front();
        static_imu_init_ptr->AddIMU(cur_imu_data);
        if (static_imu_init_ptr->InitSuccess())
        {
            LOG(INFO) << " static imu init ok ";
            EskfOptions eskf_option;
            eskf_option.gyro_var_ = sqrt(static_imu_init_ptr->GetCovGyro()[0]);
            eskf_option.acce_var_ = sqrt(static_imu_init_ptr->GetCovAcce()[0]);
            eskf_ptr->SetInitialConditions(eskf_option, static_imu_init_ptr->GetInitBg(),
                                           static_imu_init_ptr->GetInitBa(), static_imu_init_ptr->GetGravity());
            has_init_eskf = true;
            return true;
        }
        imu_buff.pop_front();
    }
    return static_imu_init_ptr->InitSuccess();
}

}  // namespace LocUtils
