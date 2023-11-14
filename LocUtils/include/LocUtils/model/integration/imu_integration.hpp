//
// Created by tian on 2023/8/8 .
//

#pragma once

#include "LocUtils/common/eigen_types.h"
#include "LocUtils/sensor_data/imu_data.hpp"

namespace LocUtils
{
    class ImuIntegration
    {
        public:
            ImuIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba);
            ~ImuIntegration();

            void AddImu(const IMU& imu);
        private:
            // 累计量
            SO3 R_;
            Vec3d v_ = Vec3d::Zero();
            Vec3d p_ = Vec3d::Zero();

            double timestamp_ = 0.0;

            // 零偏，由外部设定
            Vec3d bg_ = Vec3d::Zero();
            Vec3d ba_ = Vec3d::Zero();

            Vec3d gravity_ = Vec3d(0, 0, -9.8);  // 重力

    };

}