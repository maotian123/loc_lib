//
// Created by Tian on 2023/09/12.
//

#include <glog/logging.h>

#include "LocUtils/model/matching/3d/ndt/ndt_registration.hpp"

namespace LocUtils
{
    NdtRegistration::NdtRegistration(): 
                target_(new PointCloudType)
                ,source_(new PointCloudType)
    {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
        PrintNdtMethodUse();
    }

    NdtRegistration::NdtRegistration(NdtOptions options): 
                options_(options)
                ,target_(new PointCloudType)
                ,source_(new PointCloudType)
    {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
        PrintNdtMethodUse();
    }

    void NdtRegistration::SetSource(const CloudPtr &source)
    {
        if(options_.method_ == NdtMethod::PCL_NDT)
        {
            //TODO
            return;
        }

        *source_ = *source;

        return;
    }

    bool NdtRegistration::CaculateMatrixHAndB(const CloudPtr &input_source, 
                                     const SE3& predict_pose, 
                                     Mat6d &H, 
                                     Vec6d &B)
    {

    }

    void NdtRegistration::GenerateNearbyGrids()
    {
        if (options_.nearby_type_ == NdtNearbyType::CENTER) {
            nearby_grids_.emplace_back(NdtKeyType::Zero());
        } else if (options_.nearby_type_ == NdtNearbyType::NEARBY6) 
        {
            nearby_grids_ = {NdtKeyType(0, 0, 0),  NdtKeyType(-1, 0, 0), NdtKeyType(1, 0, 0), NdtKeyType(0, 1, 0),
                            NdtKeyType(0, -1, 0), NdtKeyType(0, 0, -1), NdtKeyType(0, 0, 1)};
        }
        int type = (int)options_.nearby_type_;
        LOG(INFO) << " nearby_grids_ " << nearby_grids_.size();
        LOG(INFO) << " type " << type;
    }

    bool NdtRegistration::SetInputTarget(const CloudPtr &input_target)
    {
        switch (options_.method_)
        {
            case NdtMethod::PCL_NDT:    
                break;
            case NdtMethod::DIRECT_NDT:
                if(SetDirectNdtTargetCloud(input_target))
                    return true;
                break;
            case NdtMethod::INCREMENTAL_NDT:
                if(SetIncNdtTargetCloud(input_target))
                    return true;
                break;
            default: 
                break;
        }

        return true;

    }

    bool NdtRegistration::SetDirectNdtTargetCloud(const CloudPtr &input_target)
    {
        assert(input_target != nullptr);
        assert(input_target->empty() == false);
        *target_ = *input_target;
        grids_.clear();
        LOG(INFO) << "set direct input ndt target cloud ";
        /// 分配体素
        std::vector<size_t> index(target_->size());
        std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

        std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
            auto pt = ToVec3d(target_->points[idx]);
            auto key = (pt * options_.inv_voxel_size_).cast<int>();
            if (grids_.find(key) == grids_.end()) {
                grids_.insert({key, {idx}});
            } else {
                grids_[key].idx_.emplace_back(idx);
            }
        });

        /// 计算每个体素中的均值和协方差
        std::for_each(grids_.begin(), grids_.end(), [this](auto& v)
        {
            if (v.second.idx_.size() > options_.min_pts_in_voxel_) 
            {
                // 进行均值和协方差计算的代码逻辑
                math::ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                                    [this](const size_t& idx) { return ToVec3d(target_->points[idx]); });

                // SVD 检查最大与最小奇异值，限制最小奇异值
                Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Vec3d lambda = svd.singularValues();
                if (lambda[1] < lambda[0] * 1e-3) {
                    lambda[1] = lambda[0] * 1e-3;
                }

                if (lambda[2] < lambda[0] * 1e-3) {
                    lambda[2] = lambda[0] * 1e-3;
                }

                Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
                
                v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
                
            }
        });

        /// 删除点数不够的
        for (auto iter = grids_.begin(); iter != grids_.end();) {
            if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
                iter++;
            } else {
                iter = grids_.erase(iter);
            }
        }

        // 计算点云中心
        // target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
        //                                 [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
        //                 target_->size();
    }

    bool NdtRegistration::SetIncNdtTargetCloud(const CloudPtr &input_target) {
        std::set<NdtKeyType, less_vec<3>> active_voxels;  // 记录哪些voxel被更新
        for (const auto& p : input_target->points) {
            auto pt = ToVec3d(p);
            auto key = (pt * options_.inv_voxel_size_).cast<int>();
            auto iter = inc_grids_.find(key);
            if (iter == inc_grids_.end()) {
                // 栅格不存在
                data_.push_front({key, {pt}});
                inc_grids_.insert({key, data_.begin()});

                if (data_.size() >= options_.capacity_) {
                    // 删除一个尾部的数据
                    inc_grids_.erase(data_.back().first);
                    data_.pop_back();
                }
            } else {
                // 栅格存在，添加点，更新缓存
                iter->second->second.AddPoint(pt);
                data_.splice(data_.begin(), data_, iter->second);  // 更新的那个放到最前
                iter->second = data_.begin();                      // grids时也指向最前
            }

            active_voxels.emplace(key);
        }

        // 更新active_voxels
        std::for_each( active_voxels.begin(), active_voxels.end(),
                    [this](const auto& key) { UpdateVoxel(inc_grids_[key]->second); });   


        flag_first_scan_  = true;   
        return true;      
    }

    void NdtRegistration::UpdateVoxel(NdtVoxelData& v) {
        if (flag_first_scan_) {
            if (v.pts_.size() > 1) {
                math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& p) { return p; });
                v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免出nan
            } else {
                v.mu_ = v.pts_[0];
                v.info_ = Mat3d::Identity() * 1e2;
            }

            v.ndt_estimated_ = true;
            v.pts_.clear();
            return;
        }

        if (v.ndt_estimated_ && v.num_pts_ > options_.max_pts_in_voxel_) {
            return;
        }

        if (!v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
            // 新增的voxel
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& p) { return p; });
            v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免出nan
            v.ndt_estimated_ = true;
            v.pts_.clear();
        } else if (v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
            // 已经估计，而且还有新来的点
            Vec3d cur_mu, new_mu;
            Mat3d cur_var, new_var;
            math::ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vec3d& p) { return p; });
            math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);

            v.mu_ = new_mu;
            v.sigma_ = new_var;
            v.num_pts_ += v.pts_.size();
            v.pts_.clear();

            // check info
            Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }

            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
            v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    }

    bool NdtRegistration::ScanMatch(const CloudPtr& input_source, 
                                const SE3& predict_pose, 
                                CloudPtr& result_cloud_ptr,
                                SE3& result_pose)
    {
        SetSource(input_source);
        switch (options_.method_)
        {
            case NdtMethod::PCL_NDT:
                break;
            case NdtMethod::DIRECT_NDT:
                AlignNdt(predict_pose, result_pose);
                break;
            case NdtMethod::INCREMENTAL_NDT:
                AlignIncNdt(predict_pose,result_pose);
                break;
            default:
                break;
        }

        pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose.matrix().cast<float>());

        return true;
    }
    bool NdtRegistration::AlignIncNdt(const SE3& init_pose, SE3& result_pose)
    {
        // LOG(INFO) << "aligning with inc ndt, pts: " << source_->size() << ", grids: " << inc_grids_.size();
        assert(inc_grids_.empty() == false);

        SE3 pose = init_pose;

        // 对点的索引，预先生成
        int num_residual_per_point = 1;
        if (options_.nearby_type_ == NdtNearbyType::NEARBY6) {
            num_residual_per_point = 7;
        }

        std::vector<int> index(source_->points.size());
        for (int i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        // 我们来写一些并发代码
        int total_size = index.size() * num_residual_per_point;
        
        for (int iter = 0; iter < options_.max_iteration_; ++iter) {
            std::vector<bool> effect_pts(total_size, false);
            std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
            std::vector<Vec3d> errors(total_size);
            std::vector<Mat3d> infos(total_size);

            // gauss-newton 迭代
            // 最近邻，可以并发
            std::for_each(index.begin(), index.end(), [&](int idx) {
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = pose * q;  // 转换之后的q

                // 计算qs所在的栅格以及它的最近邻栅格
                Vec3i key = (qs * options_.inv_voxel_size_).cast<int>();

                for (int i = 0; i < nearby_grids_.size(); ++i) {
                    Vec3i real_key = key + nearby_grids_[i];
                    auto it = inc_grids_.find(real_key);
                    int real_idx = idx * num_residual_per_point + i;
                    /// 这里要检查高斯分布是否已经估计
                    if (it != inc_grids_.end() && it->second->second.ndt_estimated_) {
                        auto& v = it->second->second;  // voxel
                        Vec3d e = qs - v.mu_;

                        // check chi2 th
                        double res = e.transpose() * v.info_ * e;
                        if (std::isnan(res) || res > options_.res_outlier_th_) {
                            effect_pts[real_idx] = false;
                            continue;
                        }

                        // build residual
                        Eigen::Matrix<double, 3, 6> J;
                        J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
                        J.block<3, 3>(0, 3) = Mat3d::Identity();

                        jacobians[real_idx] = J;
                        errors[real_idx] = e;
                        infos[real_idx] = v.info_;
                        effect_pts[real_idx] = true;
                    } else {
                        effect_pts[real_idx] = false;
                    }
                }
            });

            // 累加Hessian和error,计算dx
            double total_res = 0;

            int effective_num = 0;

            Mat6d H = Mat6d::Zero();
            Vec6d err = Vec6d::Zero();

            for (int idx = 0; idx < effect_pts.size(); ++idx) {
                if (!effect_pts[idx]) {
                    continue;
                }

                total_res += errors[idx].transpose() * infos[idx] * errors[idx];
                effective_num++;

                H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
                err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
            }

            if (effective_num < options_.min_effective_pts_) {
                LOG(WARNING) << "effective num too small: " << effective_num;
                result_pose = pose;
                return false;
            }
            
            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            // 更新
            // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
            //         << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
            //         << ", dx: " << dx.transpose();

            if (dx.norm() < options_.eps_) {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        result_pose = pose;
        return true;
    }

    bool NdtRegistration::AlignNdt(const SE3& init_pose, SE3& result_pose) 
    {
        assert(grids_.empty() == false);
        // LOG(INFO) << "aligning with ndt";

        SE3 predict_pose = init_pose;
        if (options_.remove_centroid_) 
        {
            predict_pose.translation() = target_center_ - source_center_;  // 设置平移初始值
            LOG(INFO) << "init trans set to " << predict_pose.translation().transpose();
        }

        // 对点的索引，预先生成
        int num_residual_per_point = 1;
        if (options_.nearby_type_ == NdtNearbyType::NEARBY6) {
            num_residual_per_point = 7;
        }

        for (int iter = 0; iter < options_.max_iteration_; ++iter) 
        {
            // LOG(INFO) << "iter " << iter;
            size_t effective_num = 0;
            double total_res = 0;
            Mat6d H{Mat6d::Zero()};
            Vec6d err{Vec6d::Zero()};
            for (int i{0}; i < source_->points.size(); ++i)
            {

                auto q = ToVec3d(source_->points[i]);
                Vec3d qs = predict_pose * q;  // 转换之后的q
                
                 // 计算qs所在的栅格以及它的最近邻栅格
                Vec3i key = (qs * options_.inv_voxel_size_).cast<int>();
                for (int i = 0; i < nearby_grids_.size(); ++i) 
                {
                    auto key_off = key + nearby_grids_[i];
                    auto it = grids_.find(key_off);
                    if (it != grids_.end()) 
                    {
                        auto& v = it->second;  // voxel
                        Vec3d e = qs - v.mu_;

                        // check chi2 th
                        double res = e.transpose() * v.info_ * e;
                        if (std::isnan(res) || res > options_.res_outlier_th_) 
                        {
                            continue;
                        }
                        Eigen::Matrix<double, 3, 6> J {Eigen::Matrix<double, 3, 6>::Zero()};
                        J.block<3, 3>(0, 0) = -predict_pose.so3().matrix() * SO3::hat(q);
                        J.block<3, 3>(0, 3) = Mat3d::Identity();

                        H += J.transpose() * J;
                        err += -J.transpose() * e;

                        total_res += e.dot(e);
                    }
                }
                effective_num++;   
            }

            if (H.determinant() == 0)
                return false;

            if (effective_num < options_.min_effective_pts_)
            {
                LOG(WARNING) << "effective num too small: " << effective_num;
                continue;
            }
            
            //得到位姿 标准的 A * deltax = b 求 deltax 的矩阵方程 为什么是 deltax 需要去看下视觉slam14讲中的第四\五讲
            Vec6d dx = H.inverse() * err;
            //deltax 的含义就是 把原始的预测变量predic_pose叠加 deltax 能够得到最小的err 
            //最小的erro代表配准的误差最小
            predict_pose.so3() = predict_pose.so3() * SO3::exp(dx.head<3>());
            predict_pose.translation() += dx.tail<3>();

            // 更新
            // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
            //     << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();
            
            if (dx.norm() < options_.eps_) 
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }    
        }
        result_pose = predict_pose;

        return true;
    }

    float NdtRegistration::GetFitnessScore()
    {
        float score {0.0f};

        return score;
    }

}