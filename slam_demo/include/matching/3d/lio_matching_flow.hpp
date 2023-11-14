//
// Created by Tian on 2023/10/27.
//

#pragma once
//sub pub
#include <LocUtils/subscriber/cloud_subscriber.hpp>
#include <LocUtils/subscriber/imu_subscriber.hpp>
#include <LocUtils/subscriber/gnss_subscriber.hpp>

#include <LocUtils/publisher/cloud_publisher.hpp>
#include <LocUtils/publisher/pose_publisher.hpp>
#include <LocUtils/publisher/odometry_publisher.hpp>
//tools
#include <LocUtils/tools/parameter.hpp>
#include <LocUtils/model/sync/measure_sync.hpp>
//loc
#include <LocUtils/slam/3d/loc.hpp>

enum class MatchingMethod 
{
    LOAM = 0,
    ICP,
    NDT     
}; 

class LioMatchingFlow
{
    public:
        LioMatchingFlow(ros::NodeHandle& nh, std::string yaml_path);

        bool Run();
        
    private:
        bool ReadData();
        bool HasLidarData();
        bool HasImuData();
        bool HasGnssData();

        bool ValidData();
        bool UpdateMatching();
        bool PublishData();
        void TransData();

        bool InitPosition();
        bool InitStaticImu();
        bool InitConfig();
        bool InitSubPub();
        bool InitLoc();
    private:
        //sub pub
        std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<LocUtils::IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<LocUtils::GnssSubscriber> gnss_sub_ptr_;

        std::shared_ptr<LocUtils::CloudPublisher> cur_scan_pub_ptr_;
        std::shared_ptr<LocUtils::CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<LocUtils::CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<LocUtils::OdometryPublisher> cur_pos_pub_ptr_;

        // sync
        std::shared_ptr<LocUtils::MappingMessageSync> sync_ptr_;
        //param
        bool b_use_imu_ {false};
        bool b_use_gnss_ {false};
        bool b_use_eskf_ {false};
        bool b_static_imu_ {false};
        std::shared_ptr<LocUtils::Parameters> param_ptr_;

        LocUtils::LocOptions loc_option_;
        LocUtils::IcpOptions icp_option_;
        LocUtils::NdtOptions ndt_option_;
        LocUtils::LoamOption loam_option_;
        LocUtils::MappingSyncOptions sync_option_;
        
        MatchingMethod matching_method_ {MatchingMethod::NDT};
        LocUtils::LidarType lidar_type_ {LocUtils::LidarType::NOTYPE};

        std::string lidar_topic_ {"/rslidar_points"};
        std::string imu_topic_ {"/imu_data"};
        std::string gnss_topic_ {"/fix"};
        const std::string k_cur_scan_topic_ {"/locutils/cur_scan"};

        const std::string k_map_topic_ {"/locutils/global_map"};
        const std::string k_local_map_topic_ {"/locutils/local_map"};
        const std::string k_cur_pos_topic_ {"/locutils/curr_pose"};
        // sensor data
        std::deque<LocUtils::CloudData> cloud_data_buff_;
        std::deque<LocUtils::CloudDataFullType> full_cloud_data_buff_;
        std::deque<LocUtils::IMU>  imu_data_buff_;       
        std::deque<LocUtils::GnssData> gnss_data_buff_;

        LocUtils::CloudData curr_scan_;
        LocUtils::CloudDataFullType curr_full_scan_;
        LocUtils::IMU curr_imu_;
        LocUtils::GnssData curr_gnss_;
        //loc
        std::shared_ptr<LocUtils::Loc> loc_ptr_;
        ros::NodeHandle nh_;

        SE3 cur_pos_ {SE3()};
        SE3 init_pos_ {SE3()};

        SE3 T_gnss_imu_ {SE3()};
        
        // double origin_latitude_ {36.509905};
        // double origin_longitude_ {116.802690};
        // double origin_altitude_ {69.921005};
};