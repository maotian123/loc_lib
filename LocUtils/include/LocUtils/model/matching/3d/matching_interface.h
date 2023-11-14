//
// Created by Tian on 2023/09/04.
//
#pragma once

#include <yaml-cpp/yaml.h>
#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/point_types.h"
#include "LocUtils/common/pose_data.hpp"

namespace LocUtils
{
    class MatchingInterface 
    {
        public:
            virtual ~MatchingInterface() = default;
            
            virtual bool SetInputTarget(const CloudPtr &input_target)
            {
                return true;
            }
            //只计算H 和 B留个接口给loam
            virtual bool CaculateMatrixHAndB(const CloudPtr &input_source,
                                             const SE3& predict_pose, 
                                             Mat6d &H, 
                                             Vec6d &B)
            {
                return true;
            }
            virtual bool ScanMatch(const CloudPtr &input_source,
                                   const SE3 &predict_pose,
                                   CloudPtr &result_cloud_ptr,
                                   SE3 &result_pose)
            {
                return true;
            }
                                   
            virtual bool SetInputTarget(const CloudPtr &edge_input,const CloudPtr &surf_input)
            {
                return true;
            }
            //只计算H 和 B留个接口给loam

            virtual bool ScanMatch(const CloudPtr &edge_input,
                                   const CloudPtr &surf_input,
                                   const SE3 &predict_pose,
                                   CloudPtr &result_cloud_ptr,
                                   SE3 &result_pose)
            {
                return true;
            }

            virtual float GetFitnessScore() = 0;
    };
}
