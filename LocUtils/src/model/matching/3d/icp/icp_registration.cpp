//
// Created by Tian on 2023/09/04.
//

#include "LocUtils/model/matching/3d/icp/icp_registration.hpp"
namespace LocUtils
{
    //注入当前帧点云
    bool IcpRegistration::SetInputTarget(const CloudPtr &input_target)
    {
        if(options_.method_ == IcpMethod::PCLICP)
        {
            pcl_icp_ptr_->setInputTarget(input_target);
            return true;
        }  
        *target_ = *input_target;
        //往kdtree 搜索的类里面加入后续需要搜索的点云 为后续做准备
        kdtree_ptr_->SetTargetCloud(input_target);
        // kdtree_flann_->setInputCloud(target_);

        //这个可以暂时不官
        // target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
        //                                  [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
        //                  target_->size();
        
        // LOG(INFO) << "target center: " << target_center_.transpose();
        
        return true;
    }

    bool IcpRegistration::CaculateMatrixHAndB(const CloudPtr &input_source, 
                                              const SE3& predict_pose, 
                                              Mat6d &H, 
                                              Vec6d &B)
    {
        SetSource(input_source);
        switch (options_.method_)
        {
            case IcpMethod::P2P:
                if(!CaculateMatrixHAndBP2P(predict_pose, H, B))
                    return false;
                break;
            case IcpMethod::P2LINE:
                if(!CaculateMatrixHAndBP2Line(predict_pose, H, B))
                    return false;
                break;
            case IcpMethod::P2PLANE:
                if(!CaculateMatrixHAndBP2Plane(predict_pose, H, B))
                    return false;
                break;
            default:
                break;
        }
        return true;
    }

    bool IcpRegistration::CaculateMatrixHAndBP2P(const SE3& predict_pose, Mat6d &H, Vec6d &B)
    {
        assert(target_ != nullptr && source_ != nullptr);
        size_t effective_num = 0;
        double total_res = 0;
        for (int i{0}; i < source_->points.size(); ++i)
        {
            if (!pcl::isFinite(source_->points.at(i)))
                continue;
            auto q = ToVec3d(source_->points.at(i));

            Vec3d qs = predict_pose * q; // 转换之后的q
            //找寻当前帧的每个点云距离上一帧最近的点的索引
            std::vector<int> nn = kdtree_ptr_->FindNearstPoints(qs.template cast<float>(), 1);
            if (!nn.empty()) 
            {
                Vec3d p = ToVec3d(target_->points[nn[0]]);
                double dis2 = (p - qs).squaredNorm();
                if (dis2 > options_.max_nn_distance_) {
                    // 点离的太远了不要
                    continue;
                }

                // 构建残差方程
                effective_num++;
                Vec3d e = p - qs;
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = (predict_pose.so3().matrix() * SO3::hat(q)) / 16;
                J.block<3, 3>(0, 3) = -Mat3d::Identity();

                H += J.transpose() * J;
                B += -J.transpose() * e;

                total_res += e.dot(e);
            }
        }
        LOG(INFO) << " H " << H;
        if (effective_num < options_.min_effective_pts_)
        {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }
        //得到位姿 标准的 A * deltax = b 求 deltax 的矩阵方程 为什么是 deltax 需要去看下视觉slam14讲中的第四\五讲
        if (H.determinant() == 0)
            return false;
        return true;
    }

    bool IcpRegistration::CaculateMatrixHAndBP2Line(const SE3& predict_pose, Mat6d &H, Vec6d &B)
    {
        assert(target_ != nullptr && source_ != nullptr);
        size_t effective_num = 0;
        double total_res = 0;
        for (int i{0}; i < source_->points.size(); ++i)
        {
            auto q = ToVec3d(source_->points.at(i));
            Vec3d qs = predict_pose * q; // 转换之后的q
            std::vector<int> nn = kdtree_ptr_->FindNearstPoints(qs.template cast<float>(), 5);
            if (nn.size() == 5)
            {
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < 5; ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                Vec3d d, p0;
                if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                    // LOG(INFO) << " fit line faled";
                    continue;
                }

                effective_num++;
            
                Vec3d e = SO3::hat(d) * (qs - p0);

                if (e.norm() > options_.max_line_distance_) 
                {
                    continue;
                }

                // build residual
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = -SO3::hat(d) * predict_pose.so3().matrix() * SO3::hat(q);
                J.block<3, 3>(0, 3) = SO3::hat(d);

                H += J.transpose() * J;
                B += -J.transpose() * e;

                total_res += e.dot(e);
            }
        }
        if (effective_num < options_.min_effective_pts_)
        {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }
        if (H.determinant() == 0)
        {
            LOG(WARNING) << "H not determinant: " << effective_num;
            return false;
        }
        return true;
    }

    bool IcpRegistration::CaculateMatrixHAndBP2Plane(const SE3& predict_pose, Mat6d &H, Vec6d &B)
    {
        assert(target_ != nullptr && source_ != nullptr);
        size_t effective_num = 0;
        double total_res = 0;
        for (int i{0}; i < source_->points.size(); ++i)
        {
            auto q = ToVec3d(source_->points.at(i));
            Vec3d qs = predict_pose * q; // 转换之后的q
            std::vector<int> nn = kdtree_ptr_->FindNearstPoints(qs.template cast<float>(), 5);
            if (nn.size() > 3)
            {
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                Vec4d n;      
                if (!math::FitPlane(nn_eigen, n)) {
                    // LOG(INFO) << " fit line faled";
                    continue;
                }

                effective_num++;
                double dis = n.head<3>().dot(qs) + n[3];

                if (fabs(dis) > options_.max_plane_distance_) 
                {
                    continue;
                }

                // build residual
                Eigen::Matrix<double, 1, 6> J;
                J.block<1, 3>(0, 0) = -n.head<3>().transpose() * predict_pose.so3().matrix() * SO3::hat(q);
                J.block<1, 3>(0, 3) = n.head<3>().transpose();

                H += J.transpose() * J;
                B += -J.transpose() * dis;

                total_res += dis;
            }
        }

        if (effective_num < options_.min_effective_pts_)
        {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        if (H.determinant() == 0)
            return false;
        return true;
    }

    //输入待配准的点云和target_做配准
    bool IcpRegistration::ScanMatch(const CloudPtr& input_source, 
                                const SE3& predict_pose, 
                                CloudPtr& result_cloud_ptr,
                                SE3& result_pose)
    {
        SetSource(input_source);
        //不同的方法 点到点的icp 点到线的icp 点到面的icp
        switch (options_.method_)
        {
            case IcpMethod::P2P:
                AlignP2P(predict_pose, result_pose);
                break;
            case IcpMethod::P2LINE:
                AlignP2Line(predict_pose, result_pose);
                break;
            case IcpMethod::P2PLANE:
                AlignP2Plane(predict_pose, result_pose);
                break;
            case IcpMethod::PCLICP:
                AlignPCLICP(predict_pose, result_pose);
                break;
            default:
                break;
        }

        pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose.matrix().cast<float>());
        
        return true;
    }

    float IcpRegistration::GetFitnessScore()
    {
        float score {0.0f};
        return score;
    }

    void IcpRegistration::SetSource(CloudPtr source)
    {
        if(options_.method_ == IcpMethod::PCLICP)
        {
            pcl_icp_ptr_->setInputSource(source);
            return;
        }
        *source_ = *source;

        // source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
        //                                  [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
        //                  source_->size();
        // LOG(INFO) << "source center: " << source_center_.transpose();
    }

    bool IcpRegistration::AlignP2P(const SE3& init_pose, SE3& result_pose)
    {
        // LOG(INFO) << "aligning with point to point";
        assert(target_ != nullptr && source_ != nullptr);
        SE3 predic_pose = init_pose;
        //只估计旋转的模式
        if (!options_.use_initial_translation_) 
        {
            predic_pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        }
        // // 接下里的代码配合 书 7.2 
        // //3 x 6的矩阵
 
        for (int iter = 0; iter < options_.max_iteration_; ++iter) 
        {
            Mat6d H {Mat6d::Zero()};
            Vec6d err {Vec6d::Zero()};
            if(CaculateMatrixHAndBP2P(predic_pose, H, err))
            {
                LOG(INFO) << H.inverse();
                Vec6d dx = H.inverse()/16 * err;
                predic_pose.so3() = predic_pose.so3() * SO3::exp(dx.head<3>());
                predic_pose.translation() += dx.tail<3>();

                // 更新
                // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                //       << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();
                
                if (dx.norm() < options_.eps_) 
                {
                    // LOG(INFO) << "converged, dx = " << dx.transpose();
                    break;
                }
            }  
        }
        result_pose = predic_pose;
    }

    bool IcpRegistration::AlignP2Line(const SE3& init_pose,SE3& result_pose)
    {
        // LOG(INFO) << "aligning with point to line";
        assert(target_ != nullptr && source_ != nullptr);
        SE3 predic_pose = init_pose;
        
        if (!options_.use_initial_translation_) 
        {
            predic_pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        }

        Eigen::Matrix<double, 3, 6> jacobians;
        
        for (int iter = 0; iter < options_.max_iteration_; ++iter) 
        {
            Mat6d H {Mat6d::Zero()};
            Vec6d err {Vec6d::Zero()};

            if(CaculateMatrixHAndBP2Line(predic_pose, H, err))
            {
                Vec6d dx = H.inverse() * err;
                predic_pose.so3() = predic_pose.so3() * SO3::exp(dx.head<3>());
                predic_pose.translation() += dx.tail<3>();

                // 更新
                // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                //       << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();
                
                if (dx.norm() < options_.eps_) 
                {
                    LOG(INFO) << "converged, dx = " << dx.transpose();
                    break;
                }
            }  
        }

        result_pose = predic_pose;
        return true;
    }

    bool IcpRegistration::AlignP2Plane(const SE3& init_pose,SE3& result_pose)
    {
        // LOG(INFO) << "aligning with point to plane";
        assert(target_ != nullptr && source_ != nullptr);
        SE3 predic_pose = init_pose;
        
        if (!options_.use_initial_translation_) 
        {
            predic_pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        }

        Eigen::Matrix<double, 3, 6> jacobians;
        
        for (int iter = 0; iter < options_.max_iteration_; ++iter) 
        {
            Mat6d H {Mat6d::Zero()};
            Vec6d err {Vec6d::Zero()};
            if(CaculateMatrixHAndBP2Plane(predic_pose, H, err))
            {
                Vec6d dx = H.inverse() * err;
                predic_pose.so3() = predic_pose.so3() * SO3::exp(dx.head<3>());
                predic_pose.translation() += dx.tail<3>();

                // 更新
                // LOG(INFO) << "iter " << iter << ", dxn: " << dx.norm();
                
                if (dx.norm() < options_.eps_) 
                {
                    // LOG(INFO) << "converged, dx = " << dx.transpose();
                    break;
                }
            }
        }

        result_pose = predic_pose;
        return true;
    }

   

    bool IcpRegistration::AlignPCLICP(const SE3& init_pose, SE3& result_pose)
    {
        CloudPtr result_cloud_ptr(new PointCloudType);
        pcl_icp_ptr_->align(*result_cloud_ptr, init_pose.matrix().cast<float>());      // 配准
        Eigen::Quaterniond q(pcl_icp_ptr_->getFinalTransformation().cast<double>().block<3, 3>(0, 0));
        q.normalize();
        result_pose.setRotationMatrix(q.toRotationMatrix());
        result_pose.translation()[0] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(0, 3);
        result_pose.translation()[1] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(1, 3);
        result_pose.translation()[2] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(2, 3);

        result_pose.matrix() = pcl_icp_ptr_->getFinalTransformation().cast<double>(); // 获取变换矩阵
        LOG(INFO) << result_pose.matrix();
        return true;
    }

}
