#pragma once

#include <Eigen/Dense>
#include "eigen_types.h"
#include "velocity_data.hpp"

namespace LocUtils
{
    class PoseData 
    {
        public:
            double time_ = 0.0;
            Eigen::Matrix4f pose_ {Eigen::Matrix4f::Identity()};

            struct 
            {
                Eigen::Vector3f v = Eigen::Vector3f::Zero();
                Eigen::Vector3f w = Eigen::Vector3f::Zero();
            } vel;
            
        public:
            Eigen::Quaternionf GetQuaternion();

            void GetVelocityData(VelocityData &velocity_data) const;

            void GetSE3(SE3 &se3) const; 

    };
}