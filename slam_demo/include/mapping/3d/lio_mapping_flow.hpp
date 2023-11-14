//
// Created by Tian on 2023/10/26.
//
#pragma once
//sub pub
#include <LocUtils/subscriber/cloud_subscriber.hpp>
#include <LocUtils/subscriber/imu_subscriber.hpp>
#include <LocUtils/subscriber/gnss_subscriber.hpp>

#include <LocUtils/publisher/cloud_publisher.hpp>
#include <LocUtils/publisher/pose_publisher.hpp>
#include <LocUtils/publisher/odometry_publisher.hpp>

// sync
#include <LocUtils/model/sync/measure_sync.hpp>
//lio 
#include <LocUtils/slam/3d/lio.hpp>
//cloud filter
#include <LocUtils/model/cloud_filter/voxel_filter.hpp>
//tools
#include <LocUtils/tools/parameter.hpp>
#include <LocUtils/tools/file_manager.hpp>
#include <LocUtils/tools/save_pose.hpp>
#include <LocUtils/common/point_cloud_utils.h>
#include <LocUtils/common/sys_utils.h>
#include <LocUtils/common/eigen_types.h>
#include <LocUtils/sensor_data/imu_data.hpp>

// #inclide 
#include <gflags/gflags.h>
#include <glog/logging.h>

enum class MatchingMethod 
{
    LOAM = 0,
    ICP,
    NDT     
}; 
class LioMappingFlow
{
    public:
        LioMappingFlow(ros::NodeHandle& nh, std::string yaml_path);

        bool Run();
        
        bool SaveGlobalMap();

        bool SaveAllKeyframePoses();

        bool SaveAllGnssPoses();
    private:
        bool ReadData();
        bool TransData();
        bool HasData();
        bool UpdateMapping();
        bool ValidData();
        bool PublishData();

        bool InitStaticImu();
        bool InitFirstPose();
        bool InitConfig();
        bool InitSubPub();
        bool InitLio();
        bool InitGNSS();
        bool InitSync();
    private:
        //sub pub
        std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<LocUtils::IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<LocUtils::GnssSubscriber> gnss_sub_ptr_;

        std::shared_ptr<LocUtils::CloudPublisher> cur_scan_pub_ptr_;
        std::shared_ptr<LocUtils::CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<LocUtils::CloudPublisher> global_map_pub_ptr_;
        
        std::shared_ptr<LocUtils::OdometryPublisher> cur_pos_pub_ptr_;
        std::shared_ptr<LocUtils::OdometryPublisher> gnss_pos_pub_ptr_;
        //tools
        std::shared_ptr<LocUtils::Parameters> param_ptr_;

        // sensor data
        std::deque<LocUtils::CloudData> cloud_data_buff_;
        std::deque<LocUtils::CloudDataFullType> full_cloud_data_buff_;

        std::deque<LocUtils::IMU> imu_data_buff_;
        std::deque<LocUtils::GnssData> gnss_data_buff_;
        std::deque<SE3> allframe_gnss_pose_buff_;
        
        LocUtils::CloudData curr_scan_;
        LocUtils::CloudDataFullType curr_full_scan_;
        LocUtils::IMU curr_imu_;
        LocUtils::GnssData curr_gnss_;
        // lio 
        std::shared_ptr<LocUtils::Lio> lio_ptr_;
        // sync
        std::shared_ptr<LocUtils::MappingMessageSync> sync_ptr_;
        //param
        LocUtils::IcpOptions icp_option_;
        LocUtils::NdtOptions ndt_option_;
        LocUtils::LioOptions lio_option_;
        LocUtils::LoamOption loam_option_;
        LocUtils::MappingSyncOptions sync_option_;

        MatchingMethod matching_method_ {MatchingMethod::LOAM};
        LocUtils::LidarType lidar_type_ {LocUtils::LidarType::NOTYPE};

        bool b_static_imu_ {false};
        
        bool b_use_eskf_ {false};
        bool b_use_imu_  {false};
        bool b_use_gnss_ {false};
        bool sensor_inited_flag_ {false};

        //topics
        std::string lidar_topic_ {"/rslidar_points"};
        std::string imu_topic_ {"/gps/imu"};
        std::string gnss_topic_ {"/gps/fix"};
        
        const std::string k_cur_scan_topic_ {"/locutils/cur_scan"};
        const std::string k_local_map_topic_ {"/locutils/local_map"};
        const std::string k_map_topic_ {"/locutils/global_map"};
        const std::string k_cur_pos_topic_ {"/locutils/curr_pose"};
        const std::string k_cur_gnss_pos_topic_ {"/locutils/curr_gnss_pose"};

        ros::NodeHandle nh_;
        //pose data
        SE3 init_pose_ {SE3()};
        SE3 cur_pose_ {SE3()};
        // gps坐标
        SE3 gnss_pose_ {SE3()};
        SE3 T_gnss_imu_ {SE3()};
        Mat4d T_imu_lidar_  {Mat4d::Identity()};

        const float k_SYNC_TIME_LIMT_{0.05};
};