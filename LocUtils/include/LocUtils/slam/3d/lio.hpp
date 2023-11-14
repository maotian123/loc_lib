//
// Created by Tian on 2023/09/17.
//
#pragma once

#include "LocUtils/model/sync/measure_sync.hpp"
// eskf lio 需要进行imu初始化
#include <LocUtils/model/static_init/static_imu_init.h>
#include <LocUtils/model/eskf/eskf.hpp>

#include "LocUtils/model/matching/3d/icp/icp_registration.hpp"
#include "LocUtils/model/matching/3d/ndt/ndt_registration.hpp"
#include "LocUtils/model/matching/3d/loam/loam_registration.hpp"

#include "LocUtils/model/feature_extract/loam_feature_extract.hpp"
#include "LocUtils/model/cloud_filter/voxel_filter.hpp"

#include "LocUtils/tools/file_manager.hpp"
#include "LocUtils/tools/tic_toc.hpp"

#include "LocUtils/common/common.h"
#include "LocUtils/common/math_utils.h"

#include "LocUtils/common/point_cloud_utils.h"

#include <pcl/io/pcd_io.h>
#include <thread>

namespace LocUtils
{
    struct LioOptions
    {
        LioOptions() {}
        double kf_distance_ {0.5};            // 关键帧距离
        double kf_angle_deg_ {30};            // 旋转角度
        double gloal_map_filter_ {0.5};
        double local_map_filter_ {0.5};
        double cur_scan_filter_ {1.0};
        int num_kfs_in_local_map_ {10};       // 局部地图含有多少个关键帧
        bool display_realtime_cloud_ {true};  // 是否显示实时点云
        
        std::string key_frame_path_ {"/tmp/key_frames/"};
        std::string loam_key_frame_path_ {"/tmp/key_frames_loam/"};
        std::string map_path_ {"/tmp/map_path/"};
        std::string trajectory_path_ {"/tmp/trajectory/"};
        bool b_debug_info{true};
    
        std::vector<double> ext_t {0.0, 0.0, 0.0};
        std::vector<double> ext_r {0.0, 0.0, 0.0};

    };

    class Lio
    {
        public:
            Lio();
            Lio(LioOptions lio_option);
            Lio(LioOptions lio_option, NdtOptions ndt_option);
            Lio(LioOptions lio_option, IcpOptions icp_option);
            Lio(LioOptions lio_option, LoamOption loam_option);
            
            bool InitImu(std::deque<IMU> &imu_buff);

            void AddImu();
            void SetInitPose(const SE3 &pose);
            // lio 配搭配准
            void AddCloud(CloudPtr scan, SE3& pose);
            // loam的lio
            void AddCloud(FullCloudPtr scan, SE3& pose);

            // eskf lio pose pos为imu坐标下 tips:但是最后从lio里获取的姿态是T_w_lidar
            void AddMeasure(const MappingMeasureGroup &measure, SE3& pose);
            //获取局部地图
            bool GetLocalMap(CloudPtr& local_map);
            //获取当前点云地图
            bool GetCurrentScan(CloudPtr& current_scan);
            //获取最新关键帧位姿态(未测试)
            bool GetLatestKeyFramePose(SE3& keyframe_pose);
            //获取所有关键帧位姿态(已测试)
            bool GetAllKeyFramePose(std::shared_ptr<std::deque<SE3>>& keyframe_poses_ptr);
            //获取直接法全局地图
            bool GetGlobalMap(CloudPtr& global_map);
            //获取loam全局地图
            bool GetGlobalMap(CloudPtr& edge_map, CloudPtr& surf_map);
            //存储全局地图
            bool SaveGlobalMap(const std::string file_name);
            bool SaveGlobalMap(const std::string file_name, CloudPtr &global_map);
        private:
            /// 直接法与local map进行配准
            SE3 AlignWithLocalMap(CloudPtr source_cloud);
            /// loam系与local map进行配准
            SE3 AlignWithLocalMap(CloudPtr edge_cloud, CloudPtr surf_cloud);

            
            /// 判定是否为关键帧
            bool IsKeyframe(const SE3& current_pose);
            
            void InitEskf();
            void InitDataPath();
            // 初始化点云滤波
            void InitCloudFilter();

            void SaveKeyFrame(CloudPtr scan, const size_t& idx);
            void SaveKeyFrame(CloudPtr edge_scan, CloudPtr surf_scan, const size_t& idx);

        private:
            LioOptions lio_option_;
            NdtOptions ndt_option_;
            IcpOptions icp_option_;
            LoamOption loam_option_;

            std::shared_ptr<MatchingInterface> match_ptr_;

            std::shared_ptr<CloudFilterInterface> cur_scan_filter_ptr_;
            std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
            std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
            std::shared_ptr<LoamFeatureExtract> loam_feature_ptr_;

            SE3 last_kf_pose_ {SE3()};                  // 上一关键帧的位姿/也是作为初始帧的位姿
            std::deque<CloudPtr> scans_in_local_map_;
            
            std::deque<CloudPtr> edge_scans_in_local_map_;
            std::deque<CloudPtr> surf_scans_in_local_map_;

            CloudPtr local_map_ {nullptr};
            CloudPtr current_scan_ {nullptr};
            
            CloudPtr local_map_edge_ {nullptr};
            CloudPtr local_map_surf_ {nullptr};
            CloudPtr current_edge_scan_ {nullptr};
            CloudPtr current_surf_scan_ {nullptr};

            std::deque<SE3> estimated_poses_;

            bool b_use_loam_ {false};
            //save map thread
            std::thread save_map_thread_;
            std::atomic<bool> save_map_thread_flag_ {false};

            //imu eskf
            std::shared_ptr<LocUtils::StaticIMUInit> static_imu_init_ptr_{nullptr};
            std::shared_ptr<LocUtils::ESKFD> eskf_ptr_;
            bool has_init_eskf_ {false};
            
            SE3 TIL_ {SE3()};
            SE3 T_w_i_eskf_ {SE3()};
    };
}