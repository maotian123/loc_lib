//
// Created by Tian on 2023/08/27.
//

#include <LocUtils/subscriber/cloud_subscriber.hpp>
#include <LocUtils/subscriber/gnss_subscriber.hpp>
#include <LocUtils/subscriber/imu_subscriber.hpp>

#include <LocUtils/publisher/cloud_publisher.hpp>
#include <LocUtils/publisher/marker_publisher.hpp>
#include <LocUtils/publisher/pose_publisher.hpp>

#include <LocUtils/model/matching/3d/icp/icp_registration.hpp>
#include <LocUtils/model/matching/3d/ndt/ndt_registration.hpp>

#include <LocUtils/model/cloud_filter/voxel_filter.hpp>
#include <LocUtils/model/feature_extract/loam_feature_extract.hpp>

#include <LocUtils/model/loop_closure/scan_context.hpp>

#include <LocUtils/model/sync/measure_sync.hpp>

#include <LocUtils/slam/3d/lio.hpp>

#include <LocUtils/model/search_point/bfnn/bfnn.h>
#include <LocUtils/model/search_point/kdtree/kdtree.h>

#include <LocUtils/common/gnss_utils.hpp>
#include <LocUtils/common/point_cloud_utils.h>
#include <LocUtils/common/sys_utils.h>
#include <LocUtils/common/eigen_types.h>
#include <LocUtils/common/pose_data.hpp>

#include <LocUtils/tools/parameter.hpp>
#include <LocUtils/tools/save_pose.hpp>

#include <LocUtils/sensor_data/gnss_data.hpp>

#include <pcl/io/pcd_io.h>
#include <gflags/gflags.h>

#include <glog/logging.h>

void TestLoamExtract(int argc, char *argv[])
{
    ros::init(argc, argv, "test_matching_node");
    ros::NodeHandle nh;
    LocUtils::LoamFeatureOptions loam_feature_option;
    using LidarSubType = ::rslidar_ros::Point;

    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr 
                        = std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000, LocUtils::LidarType::ROBOSENSE);

    std::shared_ptr<LocUtils::CloudPublisher> cur_scan_pub_ptr 
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/curr_scan", "/map", 10000);

    std::shared_ptr<LocUtils::CloudPublisher> cur_edge_pub_ptr 
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/cur_edge_scan", "/map", 10000);
    
    std::shared_ptr<LocUtils::CloudPublisher> cur_surf_pub_ptr 
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/cur_surf_scan", "/map", 10000);
    loam_feature_option.num_scan_ = 32;
    std::shared_ptr<LocUtils::LoamFeatureExtract> loam_feature_ptr
                        = std::make_shared<LocUtils::LoamFeatureExtract>(loam_feature_option);

    std::deque<LocUtils::CloudDataFullType> cloud_data_buff;
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        cloud_sub_ptr->ParseData(cloud_data_buff);
        if(!cloud_data_buff.empty())
        {
            auto curr_cloud_data = cloud_data_buff.front();
            
            LocUtils::CloudPtr surf_cloud(new LocUtils::PointCloudType);
            LocUtils::CloudPtr edge_cloud(new LocUtils::PointCloudType);

            loam_feature_ptr->Extract(curr_cloud_data.cloud_ptr, surf_cloud, edge_cloud);
            cur_edge_pub_ptr->Publish(surf_cloud);
            cur_surf_pub_ptr->Publish(edge_cloud);
            cloud_data_buff.pop_front();
        }
        rate.sleep();
    }
}
//测试线程，没写服务搞了个线程检测CIN进行地图发布
void DetectKeyBoard(std::shared_ptr<LocUtils::Lio> lio_ptr)
{
    std::string strcin;
    ros::Rate r(10);
    while (ros::ok())
    {
        std::cin >> strcin;
        if (strcin == "map")
        {
            lio_ptr->SaveGlobalMap("global_map");
        }
        r.sleep();
    }
}

void TestLio(int argc, char *argv[])
{
    ros::init(argc, argv, "test_matching_node");
    ros::NodeHandle nh;
    LocUtils::IcpOptions icp_option;
    LocUtils::LioOptions lio_option;
    LocUtils::NdtOptions ndt_option;

    LocUtils::LoamOption loam_option;
    
    {
        icp_option.method_ = LocUtils::IcpMethod::P2P;
    }

    {
        loam_option.surf_icp_option_.use_ann = true;
        loam_option.edge_icp_option_.use_ann = true;
        // icp_option.method_ = LocUtils::IcpMethod::P2LINE;
    }
    
    LocUtils::LidarType lidar_type {LocUtils::LidarType::ROBOSENSE};
    bool use_loam {false};
    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr; 
    
    // using LidarSubType = LocUtils::PointType;
    if(use_loam)
    {
        cloud_sub_ptr = std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000, lidar_type);
    }
    else
    {
        cloud_sub_ptr = std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000);
    }             

    std::shared_ptr<LocUtils::CloudPublisher> local_map_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/local_map", "/map", 10000);

    // std::shared_ptr<LocUtils::CloudPublisher> global_map_pub_ptr
    //                     = std::make_shared<LocUtils::CloudPublisher>(nh, "/global_map", "/map", 10000);

    std::shared_ptr<LocUtils::CloudPublisher> cur_scan_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/curr_scan", "/map", 10000);

    std::shared_ptr<LocUtils::CloudPublisher> surf_scan_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/surf_scan", "/map", 10000);
    
    std::shared_ptr<LocUtils::CloudPublisher> edge_scan_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/edge_scan", "/map", 10000);

    std::shared_ptr<LocUtils::MarkerPublisher> key_frame_pub_ptr 
                        = std::make_shared<LocUtils::MarkerPublisher>(nh, "/keyframes", "/map", 10000);

    std::shared_ptr<LocUtils::PosePublisher> cur_pose_pub_ptr 
                        = std::make_shared<LocUtils::PosePublisher>(nh, "/curr_pose", "/map", 10000);

    std::shared_ptr<LocUtils::LoamFeatureExtract> loam_feature_extract_ptr 
                        = std::make_shared<LocUtils::LoamFeatureExtract>(loam_option.feature_option_);

    std::shared_ptr<LocUtils::Lio> lio_ptr_;

    if(use_loam)
    {
        lio_ptr_ = std::make_shared<LocUtils::Lio>(lio_option, loam_option);
    }
    else
    {
        lio_ptr_ = std::make_shared<LocUtils::Lio>(lio_option, icp_option);   
    }
                        

    std::shared_ptr<LocUtils::CloudFilterInterface> cur_scan_filter_ptr = std::make_shared<LocUtils::VoxelFilter>(0.5);

    LocUtils::CloudPtr cur_scan (new LocUtils::PointCloudType());
    LocUtils::CloudPtr local_map (new LocUtils::PointCloudType());
    std::shared_ptr<std::vector<SE3>> key_frame_poses_ptr = std::make_shared<std::vector<SE3>>();

    // std::thread detectKeyProcess(DetectKeyBoard, lio_ptr_);
    // detectKeyProcess.detach();

    ros::Rate rate(10);

    std::deque<LocUtils::CloudData> cloud_data_buff;
    std::deque<LocUtils::CloudDataFullType> cloud_full_data_buff;
    LocUtils::CloudPtr pub_cloud_ptr(new LocUtils::PointCloudType());
    SE3 T_w_lc;

    lio_ptr_->SetInitPose(SE3());
    while (ros::ok())
    {
        ros::spinOnce();
        if(use_loam)
        {
            cloud_sub_ptr->ParseData(cloud_full_data_buff);
        }
        else
        {
            cloud_sub_ptr->ParseData(cloud_data_buff);
        }

        while(cloud_data_buff.size() > 0 || cloud_full_data_buff.size() > 0)
        {
            SE3 result_pos;
            if(use_loam)
            {
                LocUtils::CloudDataFullType cur_scan_data = cloud_full_data_buff.front();
                lio_ptr_->AddCloud(cur_scan_data.cloud_ptr, result_pos);
            }
            else
            {
                LocUtils::CloudData cur_scan_data = cloud_data_buff.front();
                lio_ptr_->AddCloud(cur_scan_data.cloud_ptr, result_pos);
            }
            LocUtils::TicToc tic;
            if (lio_ptr_->GetCurrentScan(cur_scan))
            {
                cur_scan_pub_ptr->Publish(cur_scan);
            }
            tic.tic();

            if(lio_ptr_->GetLocalMap(local_map))
            {
                local_map_pub_ptr->Publish(local_map);
            }
            LOG(INFO) << " add cloud use time " << tic.toc();

            // if(lio_ptr_->GetAllKeyFramePose(key_frame_poses_ptr))
            // {
            //     std::vector<Vec3f> key_frame;
            //     for(auto i : *key_frame_poses_ptr)
            //     {
            //         key_frame.push_back(i.translation().cast<float>());
            //         std::cout << key_frame[0].x() << " " << key_frame[0].y();
            //     }
            //     key_frame_pub_ptr->Publish(key_frame);
            // }
            cur_pose_pub_ptr->Publish(result_pos);

            if(use_loam)
            {
                cloud_full_data_buff.pop_front();
            }
            else
            {
                cloud_data_buff.pop_front();
            }

        }
        rate.sleep();
        // cloud_sub_ptr->ParseData()
    }
}

void TestIcpMatching(int argc, char *argv[])
{
    ros::init(argc, argv, "test_matching_node");
    ros::NodeHandle nh;
    LocUtils::IcpOptions icp_option;
    icp_option.method_ = LocUtils::IcpMethod::P2P;

    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr 
                        = std::make_shared<LocUtils::CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);

    std::shared_ptr<LocUtils::CloudPublisher> cloud_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/icp_cloud", "/map", 10000);

    std::shared_ptr<LocUtils::MatchingInterface> icp_match_ptr_
                        = std::make_shared<LocUtils::IcpRegistration>(icp_option);

    ros::Rate rate(10);

    std::deque<LocUtils::CloudData> cloud_data_buff;
    LocUtils::CloudPtr pub_cloud_ptr(new LocUtils::PointCloudType);
    SE3 T_w_lc;

    while(ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);

        while(cloud_data_buff.size() > 0)
        {
            static LocUtils::CloudData last_cloud = cloud_data_buff.front();
            LocUtils::CloudData current_cloud = cloud_data_buff.front();
            LocUtils::CloudData result_cloud;
            SE3 init_pos, T_lc_ln;

            // 滤波
            LocUtils::VoxelGrid(current_cloud.cloud_ptr, 0.5);
            LocUtils::VoxelGrid(last_cloud.cloud_ptr, 0.5);
            // 输入target cloud
            icp_match_ptr_->SetInputTarget(last_cloud.cloud_ptr);
            // 配准
            icp_match_ptr_->ScanMatch(current_cloud.cloud_ptr, init_pos, result_cloud.cloud_ptr, T_lc_ln);
            T_w_lc = T_w_lc * T_lc_ln;
            last_cloud = current_cloud;
            pcl::transformPointCloud(*current_cloud.cloud_ptr, *result_cloud.cloud_ptr, T_w_lc.matrix());
            
            // 建图
            *pub_cloud_ptr += *result_cloud.cloud_ptr;

            LOG(INFO) << " FILTER BEFOR " << pub_cloud_ptr->points.size();
            LocUtils::VoxelGrid(pub_cloud_ptr, 0.5);
            LOG(INFO) << " FILTER AFTER " << pub_cloud_ptr->points.size();

            cloud_pub_ptr->Publish(pub_cloud_ptr);
            cloud_data_buff.pop_front();
        }
        rate.sleep();
        // cloud_sub_ptr->ParseData()
    }
}

void TestNdtMatching(int argc, char *argv[])
{
    ros::init(argc, argv, "test_matching_node");
    ros::NodeHandle nh;
    LocUtils::NdtOptions ndt_option;

    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr 
                        = std::make_shared<LocUtils::CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);

    std::shared_ptr<LocUtils::CloudPublisher> cloud_pub_ptr 
                        = std::make_shared<LocUtils::CloudPublisher>(nh, "/icp_cloud", "/map", 10000);
    
    std::shared_ptr<LocUtils::MatchingInterface> ndt_match_ptr_
                        = std::make_shared<LocUtils::NdtRegistration>(ndt_option);

    ros::Rate rate(10);

    std::deque<LocUtils::CloudData> cloud_data_buff;
    LocUtils::CloudPtr pub_cloud_ptr(new LocUtils::PointCloudType);
    SE3 T_w_lc;
    while(ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);

        while(cloud_data_buff.size() > 0)
        {
            static LocUtils::CloudData last_cloud = cloud_data_buff.front();
            LocUtils::CloudData current_cloud = cloud_data_buff.front();
            LocUtils::CloudData result_cloud;
            SE3 init_pos, T_lc_ln;

            // 滤波
            LocUtils::VoxelGrid(current_cloud.cloud_ptr, 0.5);
            LocUtils::VoxelGrid(last_cloud.cloud_ptr, 0.5);
            // 输入target cloud
            ndt_match_ptr_->SetInputTarget(last_cloud.cloud_ptr);
            // 配准
            ndt_match_ptr_->ScanMatch(current_cloud.cloud_ptr, init_pos, result_cloud.cloud_ptr, T_lc_ln);
            T_w_lc = T_w_lc * T_lc_ln;
            last_cloud = current_cloud;
            pcl::transformPointCloud(*current_cloud.cloud_ptr, *result_cloud.cloud_ptr, T_w_lc.matrix());
            
            // 建图
            *pub_cloud_ptr += *result_cloud.cloud_ptr;

            LOG(INFO) << " FILTER BEFOR " << pub_cloud_ptr->points.size();
            LocUtils::VoxelGrid(pub_cloud_ptr, 0.5);
            LOG(INFO) << " FILTER AFTER " << pub_cloud_ptr->points.size();

            cloud_pub_ptr->Publish(pub_cloud_ptr);
            cloud_data_buff.pop_front();
        }
        rate.sleep();
        // cloud_sub_ptr->ParseData()
    }
}

void TestBFNN()
{
    std::shared_ptr<LocUtils::SearchPointInterface> search_point_ptr = std::make_shared<LocUtils::BfnnRegistration>();
    LocUtils::CloudPtr first(new LocUtils::PointCloudType);
    LocUtils::CloudPtr second(new LocUtils::PointCloudType);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/first.pcd", *first);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/second.pcd", *second);
    if(first->empty() || second->empty())
    {
        LOG(ERROR) << "NO CLOUD DATA";
        return;
    }
    LocUtils::VoxelGrid(first);
    LocUtils::VoxelGrid(second);
    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    LocUtils::evaluate_and_call(
        [&first, &second, &search_point_ptr]()
        {
            search_point_ptr->SetTargetCloud(first);
            std::vector<int> index_buff = search_point_ptr->FindNearstPoints(
                LocUtils::ToVec3f(second->points[0]) , 1
                );
            double aaaa = (LocUtils::ToVec3f(first->points.at(index_buff[0])) - LocUtils::ToVec3f(second->points[0])).squaredNorm(); 
            LOG(INFO) << "AAAA " << aaaa;
        },
        "暴力搜索点", 10
    );
    return;
}

void TestKdtree()
{
    std::shared_ptr<LocUtils::SearchPointInterface> search_point_ptr = std::make_shared<LocUtils::KdtreeRegistration>();
    LocUtils::CloudPtr first(new LocUtils::PointCloudType);
    LocUtils::CloudPtr second(new LocUtils::PointCloudType);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/first.pcd", *first);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/second.pcd", *second);
    if(first->empty() || second->empty())
    {
        LOG(ERROR) << "NO CLOUD DATA";
        return;
    }
    LocUtils::VoxelGrid(first);
    LocUtils::VoxelGrid(second);
    LOG(INFO) << "points: " << first->size() << ", " << second->size();
    
    LocUtils::evaluate_and_call(
        [&first, &second, &search_point_ptr]()
        {
            search_point_ptr->SetTargetCloud(first);
            std::vector<int> index_buff = search_point_ptr->FindNearstPoints(
                LocUtils::ToVec3f(second->points[1]) , 5
                );
            double aaaa = (LocUtils::ToVec3f(first->points.at(index_buff[0])) - LocUtils::ToVec3f(second->points[1])).squaredNorm(); 
            LOG(INFO) << "AAAA " << aaaa;
        },
        "kdtee 搜索 ", 10
    );
}

void TestMatching()
{
    LocUtils::CloudPtr first(new LocUtils::PointCloudType);
    LocUtils::CloudPtr second(new LocUtils::PointCloudType);
    LocUtils::CloudPtr result(new LocUtils::PointCloudType);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/first.pcd", *first);
    pcl::io::loadPCDFile("/root/workspace/loc_lib/data/second.pcd", *second);
    SE3 init_pos, T_lc_ln;

    LocUtils::IcpOptions icp_option;
    icp_option.method_ = LocUtils::IcpMethod::P2LINE;

    std::shared_ptr<LocUtils::MatchingInterface> icp_match_ptr_
                    = std::make_shared<LocUtils::IcpRegistration>(icp_option);

    LocUtils::VoxelGrid(first, 0.5);
    LocUtils::VoxelGrid(second, 0.5);
    // 输入target cloud
    icp_match_ptr_->SetInputTarget(first);
    // 配准
    icp_match_ptr_->ScanMatch(second, init_pos, result, T_lc_ln);
    LocUtils::SaveCloudToFile("/root/workspace/loc_lib/data/result.pcd", result);
    LOG(INFO) << "matching ok ";
}

void TestParameter(int argc, char *argv[])
{
    ros::init(argc, argv, "test_param_node");
    ros::NodeHandle nh;

    LocUtils::IcpOptions icp_option;
    LocUtils::LioOptions lio_option;
    LocUtils::LoamOption loam_option;
    icp_option.method_ = LocUtils::IcpMethod::P2LINE;
    
    LocUtils::Parameters param("/root/loc_lib/slam_demo/config/slam.yaml");
    int max_iter = param.GetParamFromYaml("lio_mapping/icp_option/max_iteration", 10);
    max_iter = param.GetParamFromYaml("lio_mapping/icp_option/max_iteration", 10);
    max_iter = param.GetParamFromYaml("lio_mapping/icp_option/max_iteration", 10);
    max_iter = param.GetParamFromYaml("lio_mapping/icp_option/max_iteration", 10);
    double max_iter2 = param.GetParamFromYaml("lio_mapping/icp_option/max_nn_distance", 2.0);
    std::string lidar_topic = param.GetParamFromYaml<std::string>("lio_matching/lidar_topic", "default");

    LOG(INFO) << "max_iter: " << max_iter << std::endl;
    LOG(INFO) << "max_nn_distance: " << max_iter2 << std::endl;
    LOG(INFO) << "lidar_topic: " << lidar_topic << std::endl;
}

//测试线程，没写服务搞了个线程检测CIN进行pose保存
void DetectKeyBoardGNSS(std::shared_ptr<LocUtils::SavePose> save_pose_ptr, 
                        std::shared_ptr<std::deque<LocUtils::PoseData>> poses_ptr)
{
    std::string strcin;
    ros::Rate r(10);
    while (ros::ok())
    {
        std::cin >> strcin;
        if (strcin == "pose")
        {
            std::cout << "Save Poses Started." << std::endl;
            std::cout << poses_ptr->back().time_ << ":" << std::endl;
            std::cout << poses_ptr->back().pose_ << std::endl; 
            save_pose_ptr->SaveAllPoseToFile(*poses_ptr);
            std::cout << "Save Poses Completed." << std::endl;
        }
        r.sleep();
    }
}
// 测试保存GNSS位姿
void TestGNSS(int argc, char *argv[])
{
    ros::init(argc, argv, "test_rtk_node");
    ros::NodeHandle nh;

    std::shared_ptr<LocUtils::GnssSubscriber> gnss_sub_ptr 
        = std::make_shared<LocUtils::GnssSubscriber>(nh, "/gps/fix", 10000);
    
    std::shared_ptr<LocUtils::IMUSubscriber> imu_sub_ptr
        = std::make_shared<LocUtils::IMUSubscriber>(nh, "/gps/imu", 10000);

    std::shared_ptr<LocUtils::PosePublisher> gnss_pose_pub_ptr
        = std::make_shared<LocUtils::PosePublisher>(nh, "/curr_pose", "/map", 10000);

    std::shared_ptr<std::deque<LocUtils::PoseData>> final_poses_ptr 
        = std::make_shared<std::deque<LocUtils::PoseData>>();

    std::shared_ptr<LocUtils::GeoToEnu> convert_ptr = std::make_shared<LocUtils::GeoToEnu>();
    std::shared_ptr<LocUtils::SavePose> save_pose_ptr
        = std::make_shared<LocUtils::SavePose>(LocUtils::SavePose::SavePoseMethod::KITTI, "/tmp/pose/gnss_pose_test.txt"); 

    std::thread detectKeyProcess(DetectKeyBoardGNSS, save_pose_ptr, final_poses_ptr);
    detectKeyProcess.detach();

    std::deque<LocUtils::GnssData> gnss_points_data_buff;
    std::deque<LocUtils::IMU> imu_data_buff;

    ros::Rate rate(10);
    Vec3d gnss_point_in_enu;
    LocUtils::PoseData pose_in_enu;
    while (ros::ok())
    {
        ros::spinOnce();

        gnss_sub_ptr->ParseData(gnss_points_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);

        while(gnss_points_data_buff.size() <= imu_data_buff.size() ? gnss_points_data_buff.size() : imu_data_buff.size())
        {
            auto temp_gnss_data = gnss_points_data_buff.front();
            if(false == convert_ptr->ConvertToENU(gnss_point_in_enu, temp_gnss_data))
            {
                LOG(INFO) << "ConvertToENU failed! KITTI/TUM file may be unavailable.";
                continue;
            }
            auto imu_data_pose = imu_data_buff.front();

            // 简单合并rtk和IMU的pose
            pose_in_enu = LocUtils::MergeToPoseData(gnss_point_in_enu, imu_data_pose, temp_gnss_data.timestamp_);//使用RTK时间戳

            final_poses_ptr->emplace_back(pose_in_enu);
            std::cout << final_poses_ptr->back().time_ << ":" << std::endl;
            std::cout << final_poses_ptr->back().pose_ << std::endl; 

            gnss_points_data_buff.pop_front(); 
            imu_data_buff.pop_front();           
        }
        std::cout << "size: " << gnss_points_data_buff.size() << ":" << imu_data_buff.size() << std::endl;
        rate.sleep();
    }
}

void TestMappingMeasureSync(int argc, char *argv[])
{
    ros::init(argc, argv, "test_sync");
    ros::NodeHandle nh;
    LocUtils::MappingSyncOptions option;

    option.behind_time_limit_ = 0.2;
    option.exced_time_limit_ = 0.02;
    option.b_sync_gnss_ = true;
    option.b_sync_imu_ = true;
    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr = std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000);

    std::shared_ptr<LocUtils::GnssSubscriber> gnss_sub_ptr 
        = std::make_shared<LocUtils::GnssSubscriber>(nh, "/gps/fix", 10000);
    
    std::shared_ptr<LocUtils::IMUSubscriber> imu_sub_ptr
        = std::make_shared<LocUtils::IMUSubscriber>(nh, "/gps/imu", 10000);

    std::shared_ptr<LocUtils::MappingMessageSync> message_sync_ptr
        = std::make_shared<LocUtils::MappingMessageSync>(option);

    std::deque<LocUtils::CloudData> cloud_data_buff;
    std::deque<LocUtils::IMU> imu_data_buff;
    std::deque<LocUtils::GnssData> gnss_data_buff;

    auto HasData = [&]() -> bool
    {
        if (cloud_data_buff.empty())
            return false;
        if (imu_data_buff.empty())  
            return false;
        if (gnss_data_buff.empty())  
            return false;

        return true;
    };

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);   

        while(HasData())
        {
            
            LocUtils::CloudData curr_cloud = cloud_data_buff.front();

            message_sync_ptr->ProcessCloud(curr_cloud);
            message_sync_ptr->ProcessImu(imu_data_buff);
            message_sync_ptr->ProcessGnss(gnss_data_buff);
            if(message_sync_ptr->IsSync())
            {
                //use imu eskf align
            }
            else
            {
                //use not guss cloud align
            }
            
            cloud_data_buff.pop_front();
            LOG(INFO) << cloud_data_buff.size();
        }

        rate.sleep();
    }
}

//测试显示sc
void TestScanContext(int argc, char *argv[])
{
    ros::init(argc, argv, "test_sc");
    ros::NodeHandle nh;

    LocUtils::LidarType lidar_type {LocUtils::LidarType::ROBOSENSE};
    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr; 
    
    // using LidarSubType = LocUtils::PointType;

    cloud_sub_ptr = std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000, lidar_type);

    std::deque<LocUtils::CloudDataFullType> cloud_full_data_buff;

    LocUtils::ScanContextOptions sc_options;
    sc_options.show_ui_flag = true;
    LocUtils::ScanContext sc_processer(sc_options);

    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_full_data_buff);

        while(cloud_full_data_buff.size() > 0)
        {
            LocUtils::CloudDataFullType cur_cloud = cloud_full_data_buff.front();
            
            sc_processer.MakeAndSaveScancontextAndKeys(cur_cloud.cloud_ptr);
            cloud_full_data_buff.pop_front();
        }
        r.sleep();
    }
}

int main(int argc, char *argv[])
{
    // LOG(INFO) << boost::filesystem::exists("/root/workspace/loc_lib/slam_demo/config/slam.yaml");
    // TestGNSS(argc, argv);
    // TestParameter(argc, argv);
    // TestLidarSub(argc, argv);
    // TestLoamExtract(argc, argv);

    // TestScanContext(argc, argv);
    // TestMappingMeasureSync(argc, argv);
    // TestLio(argc, argv);
    // TestIcpMatching(argc, argv);
    // TestMatching();
    // TestKdtree();
    return 0;
}