//
// Created by Tian on 2023/10/28.
//

#pragma once

#include "LocUtils/model/matching/3d/icp/icp_registration.hpp"
#include "LocUtils/model/matching/3d/ndt/ndt_registration.hpp"
#include "LocUtils/model/matching/3d/loam/loam_registration.hpp"

#include "LocUtils/sensor_data/imu_data.hpp"

#include "LocUtils/model/feature_extract/loam_feature_extract.hpp"
#include "LocUtils/model/cloud_filter/voxel_filter.hpp"
#include "LocUtils/model/cloud_filter/box_filter.hpp"

#include "LocUtils/tools/file_manager.hpp"
#include "LocUtils/tools/tic_toc.hpp"

#include "LocUtils/common/common.h"
#include "LocUtils/common/point_cloud_utils.h"
#include "LocUtils/common/math_utils.h"

#include "LocUtils/model/eskf/eskf.hpp"
#include "LocUtils/model/static_init/static_imu_init.h"
#include <pcl/io/pcd_io.h>
namespace LocUtils
{
    struct LocOptions
    {
        LocOptions() {}
        
        double global_map_filter_ {0.5};
        double cur_scan_filter_ {1.0};
        std::vector<float> box_filter_size_ {150.0f, 150.0f, 150.0f};
        std::string loam_surf_map_path_ {"/tmp/map_path/surf_map.pcd"};
        std::string loam_edge_map_path_ {"/tmp/map_path/edge_map.pcd"};
        std::string map_path_ {"/tmp/map_path/map.pcd"};
        std::string trajectory_path_ {"/tmp/trajectory/matching/"};

        std::vector<double> ext_t {0.0, 0.0, 0.0};
        std::vector<double> ext_r {0.0, 0.0, 0.0};
    };

    class Loc
    {
        public:
            Loc();
            Loc(LocOptions loc_option);
            Loc(LocOptions loc_option, NdtOptions ndt_option);
            Loc(LocOptions loc_option, IcpOptions icp_option);
            Loc(LocOptions loc_option, LoamOption loam_option);

            bool HasNewGlobalMap();
            bool HasNewLocalMap();
            bool HasInit();
            bool InitImu(std::deque<IMU> &imu_buff);

            //获取当前点云地图
            bool GetCurrentScan(CloudPtr& current_scan);
            CloudPtr& GetGlobalMap(); 
            CloudPtr& GetLocalMap();
            SE3 GetCurrPos();

            // 设定当前定位的初始位置
            void SetInitPose(const SE3 &pose);
            // 纯雷达定位更新姿态
            void Update(const CloudData &input_cloud);
            void Update(const CloudDataFullType &input_cloud);
            void Update(const IMU &imu_data);
        private:
            bool InitGlobalMap();
            void InitCloudFilter();
            void InitEskf();

            void AlignWithBoxMap(const CloudPtr &source_cloud);
            bool ResetLocalMap(float x, float y, float z);

        private:
            LocOptions loc_option_;
            NdtOptions ndt_option_;
            IcpOptions icp_option_;
            LoamOption loam_option_;

            std::shared_ptr<MatchingInterface> match_ptr_;
            
            std::shared_ptr<CloudFilterInterface> cur_scan_filter_ptr_;
            std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
            std::shared_ptr<BoxFilter> box_filter_ptr_;

            std::shared_ptr<LoamFeatureExtract> loam_feature_ptr_;

            //imu eskf
            std::shared_ptr<LocUtils::StaticIMUInit> static_imu_init_ptr_{nullptr};
            std::shared_ptr<LocUtils::ESKFD> eskf_ptr_;
            bool has_init_eskf_ {false};

            CloudPtr current_scan_ptr_ {nullptr};
            CloudPtr global_map_ptr_ {nullptr};
            CloudPtr local_map_ptr_ {nullptr};
            
            bool b_has_new_global_map_ {false};
            bool b_has_new_local_map_ {false};
            bool b_has_init_ {false};
            bool b_use_loam_ {false};
            
            SE3 cur_pos_ {SE3()};
            SE3 init_pos_ {SE3()};
            SE3 TIL_ {SE3()};
            SE3 T_w_i_eskf_ {SE3()};

            double cur_cloud_time_ {0.0};
    };
}