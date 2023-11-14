//
// Created by Tian on 2023/10/22.
//
#pragma once

#include <memory>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include "LocUtils/model/matching/3d/matching_interface.h"
#include "LocUtils/model/matching/3d/icp/icp_registration.hpp"

#include "LocUtils/model/search_point/kdtree/kdtree.h"
#include "LocUtils/model/search_point/bfnn/bfnn.h"

#include "LocUtils/model/feature_extract/loam_feature_extract.hpp"

namespace LocUtils
{
    
    struct LoamOption
    {
        LoamFeatureOptions feature_option_;
        IcpOptions surf_icp_option_{IcpMethod::P2PLANE};
        IcpOptions edge_icp_option_{IcpMethod::P2LINE};

        int min_edge_pts_ {20};               // 最小边缘点数
        int min_surf_pts_ {20};               // 最小平面点数
        int max_iteration_ {20};       // 最大迭代次数
        bool use_edge_points_ {true};  // 是否使用边缘点
        bool use_surf_points_ {true};  // 是否使用平面点

        double eps_ {1e-3};                 // 收敛判定条件

    };

    class LoamRegistration : public MatchingInterface
    {
        public:
            LoamRegistration();
            LoamRegistration(LoamOption option);

            bool SetInputTarget(const CloudPtr &edge_input,const CloudPtr &surf_input) override;
            
            bool ScanMatch(const CloudPtr &edge_input,
                           const CloudPtr &surf_input,
                           const SE3 &predict_pose,
                           CloudPtr &result_cloud_ptr,
                           SE3 &result_pose) override;
                                
            float GetFitnessScore() override;

        private:
            std::shared_ptr<MatchingInterface> icp_edge_ptr_;
            std::shared_ptr<MatchingInterface> icp_surf_ptr_;

            LoamOption options_;

    };
}