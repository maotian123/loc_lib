//
// Created by Tian on 2023/09/12.
//
#pragma once

#include <memory>

#include "LocUtils/common/eigen_types.h"

#include "LocUtils/model/matching/3d/matching_interface.h"
#include "LocUtils/model/search_point/kdtree/kdtree.h"
#include "LocUtils/model/search_point/bfnn/bfnn.h"

namespace LocUtils
{
    enum class NdtNearbyType 
    {
        CENTER,   // 只考虑中心
        NEARBY6,  // 上下左右前后
    };
    enum class NdtMethod
    {
        PCL_NDT, // pcl库中的ndt
        DIRECT_NDT, // 直接法
        INCREMENTAL_NDT // 增量法
    };
    struct NdtOptions {
        int max_iteration_ = 20;        // 最大迭代次数
        double voxel_size_ = 1.0;       // 体素大小
        double inv_voxel_size_ = 1.0;   //
        int min_effective_pts_ = 10;    // 最近邻点数阈值
        int min_pts_in_voxel_ = 3;      // 每个栅格中最小点数
        int max_pts_in_voxel_ = 50;    // 每个栅格中最大点数
        double eps_ = 1e-2;             // 收敛判定条件
        double res_outlier_th_ = 20.0;  // 异常值拒绝阈值
        bool remove_centroid_ = false;  // 是否计算两个点云中心并移除中心？
        size_t capacity_ = 100000;     // 缓存的体素数量

        NdtNearbyType nearby_type_ = NdtNearbyType::NEARBY6;

        NdtMethod method_ {NdtMethod::DIRECT_NDT};
    };

    using NdtKeyType = Eigen::Matrix<int, 3, 1>;  // 体素的索引

    struct NdtVoxelData {
        NdtVoxelData() {}
        NdtVoxelData(size_t id) { idx_.emplace_back(id); }
        NdtVoxelData(const Vec3d& pt) {
            pts_.emplace_back(pt);
            num_pts_ = 1;
        }
        void AddPoint(const Vec3d& pt) {
            pts_.emplace_back(pt);
            if (!ndt_estimated_) {
                num_pts_++;
            }
        }
        std::vector<Vec3d> pts_;       // 内部点，多于一定数量之后再估计均值和协方差
        bool ndt_estimated_ = false;  // NDT是否已经估计
        int num_pts_ = 0;             // 总共的点数，用于更新估计

        std::vector<size_t> idx_;      // 点云中点的索引
        Vec3d mu_ = Vec3d::Zero();     // 均值
        Mat3d sigma_ = Mat3d::Zero();  // 协方差
        Mat3d info_ = Mat3d::Zero();   // 协方差之逆
    };

    class NdtRegistration : public MatchingInterface
    {
        public:
            
            NdtRegistration();
            NdtRegistration(NdtOptions options);

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
            void PrintNdtMethodUse()
            {
                switch (options_.method_)
                {
                    case NdtMethod::PCL_NDT:
                        LOG(INFO) << " using ndt method pcl ndt ";
                        break;
                    case NdtMethod::DIRECT_NDT:
                        LOG(INFO) << " using ndt method direct ndt ";
                        break;
                    case NdtMethod::INCREMENTAL_NDT:
                        LOG(INFO) << " using ndt method incremental ndt ";
                        break;
                    default:
                        break;
                }
            }
            
            void SetSource(const CloudPtr &source);

            /// 根据最近邻的类型，生成附近网格
            void GenerateNearbyGrids();
            void UpdateVoxel(NdtVoxelData& v);
            /// 使用gauss-newton方法进行ndt配准
            bool AlignNdt(const SE3& init_pose, SE3& result_pose);
            /// 使用直接法进行ndt配准
            bool AlignIncNdt(const SE3& init_pose, SE3& result_pose);
            /// 自己维护栅格，使用直接法进行ndt配准
            bool SetIncNdtTargetCloud(const CloudPtr &input_target);
            // 直接法ndt 目标点云添加
            bool SetDirectNdtTargetCloud(const CloudPtr &input_target);
            // 增量ndt 目标点云添加
        private:
            CloudPtr target_ {nullptr};
            CloudPtr source_ {nullptr};

            NdtOptions options_;
            using KeyAndData = std::pair<NdtKeyType, NdtVoxelData>;  // 预定义
            std::list<KeyAndData> data_;                       // 真实数据，会缓存，也会清理
            std::unordered_map<NdtKeyType, std::list<KeyAndData>::iterator, hash_vec<3>> inc_grids_;  // 栅格数据，存储真实数据的迭代器
            bool flag_first_scan_ = true;  // 首帧点云特殊处理

            std::unordered_map<NdtKeyType, NdtVoxelData, hash_vec<3>> grids_;  // 栅格数据
            std::vector<NdtKeyType> nearby_grids_;

            Vec3d target_center_ = Vec3d::Zero();
            Vec3d source_center_ = Vec3d::Zero();
    };
}

