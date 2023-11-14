//
// Created by Tian on 2023/10/22.
//
#include "LocUtils/model/matching/3d/loam/loam_registration.hpp"
#include "LocUtils/tools/tic_toc.hpp"

namespace LocUtils
{
    LoamRegistration::LoamRegistration()
    {
        icp_surf_ptr_ = std::make_shared<IcpRegistration>(options_.surf_icp_option_);
        icp_edge_ptr_ = std::make_shared<IcpRegistration>(options_.edge_icp_option_);
    }

    LoamRegistration::LoamRegistration(LoamOption option)
        :options_(option)
    {   
        icp_surf_ptr_ = std::make_shared<IcpRegistration>(options_.surf_icp_option_);
        icp_edge_ptr_ = std::make_shared<IcpRegistration>(options_.edge_icp_option_);
    }

    bool LoamRegistration::SetInputTarget(const CloudPtr &edge_input,const CloudPtr &surf_input)
    {
        if(options_.use_edge_points_)
        {
            icp_edge_ptr_->SetInputTarget(edge_input);
            // LOG(INFO) << "edge input target point size " << edge_input->points.size();
        }

        if(options_.use_surf_points_)
        {
            icp_surf_ptr_->SetInputTarget(surf_input);
            // LOG(INFO) << "surf input target point size " << surf_input->points.size();
        }
        return true;
    }

    bool LoamRegistration::ScanMatch(const CloudPtr &edge_input,
                                    const CloudPtr &surf_input,
                                    const SE3 &predict_pose,
                                    CloudPtr &result_cloud_ptr,
                                    SE3 &result_pose)
    {
        // TicToc tic;
        // icp_edge_ptr_->ScanMatch(edge_input, predict_pose, result_cloud_ptr, result_pose);
        // tic.tic();
        SE3 pose = predict_pose;
        for (int iter = 0; iter < options_.max_iteration_; ++iter) 
        {
            Mat6d H {Mat6d::Zero()}, H_surf {Mat6d::Zero()}, H_edge {Mat6d::Zero()};
            Vec6d B {Vec6d::Zero()}, B_surf {Vec6d::Zero()}, B_edge {Vec6d::Zero()};
        
            if(options_.use_surf_points_)
            {

                if(!icp_surf_ptr_->CaculateMatrixHAndB(surf_input, pose, H_edge, B_edge))
                {
                    LOG(INFO) << " surf caculate fail";
                    return false;
                }
            }

            if(options_.use_edge_points_)
            {

                if(!icp_edge_ptr_->CaculateMatrixHAndB(edge_input, pose, H_surf, B_surf))
                {
                    LOG(INFO) << " edge caculate fail";
                    return false;
                }
            }
            // LOG(INFO) << "H_edge : " << H_edge << "\n"
            //           << "H_surf : " << H_surf;
            // LOG(INFO) << "B_edge : " << B_edge << "\n"
            //           << "B_surf : " << B_surf;
            H = H_edge + H_surf;
            B = B_edge + B_surf;

            Vec6d dx = H.inverse() * B;
            //deltax 的含义就是 把原始的预测变量predic_pose叠加 deltax 能够得到最小的err 
            //最小的erro代表配准的误差最小
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            if (dx.norm() < options_.eps_) 
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }
        
        result_pose = pose;
        CloudPtr cloud (new PointCloudType);
        *cloud += *edge_input;
        *cloud += *surf_input;
        pcl::transformPointCloud(*cloud, *result_cloud_ptr, result_pose.matrix().cast<float>());
        // LOG(INFO) << "USE TIME " << tic.toc();
        return true;
    }
    
    float LoamRegistration::GetFitnessScore()
    {
        return 0.0f;
    }

}