//
// Created by Tian on 2023/09/04.
//
#pragma once

#include <memory>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include "LocUtils/model/matching/3d/matching_interface.h"
#include "LocUtils/model/search_point/kdtree/kdtree.h"
#include "LocUtils/model/search_point/bfnn/bfnn.h"
namespace LocUtils
{
    enum class IcpMethod {
        P2P, //点到点
        P2LINE, //点到
        P2PLANE, //点到面
        PCLICP
    }; 

    struct IcpOptions {
        IcpOptions(){}
        IcpOptions(IcpMethod method)
        {
            method_ = method;
        }

        int max_iteration_ = 20;                // 最大迭代次数
        double max_nn_distance_ = 1.0;          // 点到点最近邻查找时阈值
        double max_plane_distance_ = 0.1;      // 平面最近邻查找时阈值
        double max_line_distance_ = 0.5;        // 点线最近邻查找时阈值
        int min_effective_pts_ = 10;            // 最近邻点数阈值
        double eps_ = 1e-2;                     // 收敛判定条件
        double euc_fitness_eps_ = 0.36;
        bool use_initial_translation_ = true;  // 是否使用初始位姿中的平移估计
        bool use_ann {false};
        IcpMethod method_{IcpMethod::P2P};
    };

    class IcpRegistration : public MatchingInterface
    {
        public:
            IcpRegistration() : 
                target_(new PointCloudType)
                ,source_(new PointCloudType)
            {

                pcl_icp_ptr_.reset(new pcl::IterativeClosestPoint<PointType, PointType>);
                // pcl_icp_ptr_->setMaxCorrespondenceDistance(options_.max_nn_distance_);
                // pcl_icp_ptr_->setTransformationEpsilon(options_.eps_);
                // pcl_icp_ptr_->setEuclideanFitnessEpsilon(options_.euc_fitness_eps_);
                // pcl_icp_ptr_->setMaximumIterations(options_.max_iteration_);
                kdtree_ptr_ = std::make_shared<KdtreeRegistration>();
            
                PrintIcpMethodUse();
            }

            IcpRegistration(IcpOptions options) : 
                options_(options)
                ,target_(new PointCloudType)
                ,source_(new PointCloudType)
                ,kdtree_flann_(new pcl::KdTreeFLANN<PointType>)
            {
                if(options.method_ == IcpMethod::PCLICP)
                {
                    pcl_icp_ptr_.reset(new pcl::IterativeClosestPoint<PointType, PointType>);
                    pcl_icp_ptr_->setMaxCorrespondenceDistance(options_.max_nn_distance_);
                    pcl_icp_ptr_->setTransformationEpsilon(options_.eps_);
                    pcl_icp_ptr_->setEuclideanFitnessEpsilon(options_.euc_fitness_eps_);
                    pcl_icp_ptr_->setMaximumIterations(options_.max_iteration_);
                }
                else
                {
                    kdtree_ptr_ = std::make_shared<KdtreeRegistration>();
                    if(options_.use_ann)
                    {
                        kdtree_ptr_->SetEnableANN(options_.use_ann, 0.1);
                    }
                }
                PrintIcpMethodUse();
            }

            bool SetInputTarget(const CloudPtr &input_target) override;
            bool CaculateMatrixHAndB(const CloudPtr &input_source, 
                                     const SE3& predict_pose, 
                                     Mat6d &H, 
                                     Vec6d &B) override;
            bool ScanMatch(const CloudPtr& input_source, 
                                const SE3& predict_pose, 
                                CloudPtr& result_cloud_ptr,
                                SE3& result_pose) override;
            float GetFitnessScore() override;
        private:
            void SetSource(CloudPtr source);

            void PrintIcpMethodUse()
            {
                switch (options_.method_)
                {
                    case IcpMethod::P2P:
                        LOG(INFO) << " using icp method p2p ";
                        break;
                    case IcpMethod::P2LINE:
                        LOG(INFO) << " using icp method p2line ";
                        break;
                    case IcpMethod::P2PLANE:
                        LOG(INFO) << " using icp method P2PLANE ";
                        break;
                    case IcpMethod::PCLICP:
                        LOG(INFO) << " using icp method PCLICP ";
                        break;  
                    default:
                        break;
                }
            }
        
            bool AlignP2P(const SE3& init_pose, SE3& result_pose);
            bool AlignP2Plane(const SE3& init_pose, SE3& result_pose);
            bool AlignP2Line(const SE3& init_pose, SE3& result_pose);
            bool AlignPCLICP(const SE3& init_pose, SE3& result_pose);
            
            bool CaculateMatrixHAndBP2P(const SE3& predict_pose, Mat6d &H, Vec6d &B);
            bool CaculateMatrixHAndBP2Plane(const SE3& predict_pose, Mat6d &H, Vec6d &B);
            bool CaculateMatrixHAndBP2Line(const SE3& predict_pose, Mat6d &H, Vec6d &B);
            
        private:
            IcpOptions options_;

            std::shared_ptr<SearchPointInterface> kdtree_ptr_;
            //上一帧的点云             
            CloudPtr target_ {nullptr};
            //输入的点云 可理解为当前点云
            CloudPtr source_ {nullptr};
            
            Vec3d target_center_ {Vec3d::Zero()};
            Vec3d source_center_ {Vec3d::Zero()};

            pcl::IterativeClosestPoint<PointType, PointType>::Ptr pcl_icp_ptr_;
            pcl::KdTreeFLANN<PointType>::Ptr kdtree_flann_;
            
    };
}