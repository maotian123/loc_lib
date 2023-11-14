//
// Created by Tian on 2023/10/26.
//
#include "mapping/3d/lio_mapping_flow.hpp"

LioMappingFlow::LioMappingFlow(ros::NodeHandle& nh, std::string yaml_path)
                              :nh_(nh)
{
    param_ptr_ = std::make_shared<LocUtils::Parameters>(yaml_path);

    InitConfig();
    InitSubPub();
    InitLio();
    InitSync();
}

bool LioMappingFlow::InitStaticImu()
{
    // 没使用 imu 或者不使用eskf就不需要initstatic imu
    if(!b_use_imu_ || !b_use_eskf_)
    {
        return true;
    }

    if(b_static_imu_)
        return true;
        
    imu_sub_ptr_->ParseData(imu_data_buff_);
    b_static_imu_ = lio_ptr_->InitImu(imu_data_buff_);
    return b_static_imu_;
}

bool LioMappingFlow::InitGNSS()
{
    if(!b_use_gnss_)
        return true;
    static bool gnss_inited = false;
    if (!gnss_inited) {
        if(sync_ptr_->GetMeasure().b_gnss_sync_success_)
        {
            LocUtils::GnssData gnss_data = sync_ptr_->GetMeasure().gnss_buff_.back();
            gnss_data.InitOriginPosition();
            gnss_inited = true;
            return true;
        }
    }

    return gnss_inited;
}

bool LioMappingFlow::InitConfig()
{
    ndt_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/max_iteration" ,20);
    ndt_option_.voxel_size_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/voxel_size" ,1.0);
    ndt_option_.min_effective_pts_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/min_effective_pts" ,10);
    ndt_option_.min_pts_in_voxel_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/min_pts_in_voxel" ,3);
    ndt_option_.max_pts_in_voxel_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/max_pts_in_voxel" ,50);
    ndt_option_.eps_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/eps" ,1e-2);
    ndt_option_.res_outlier_th_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/res_outlier_th" ,20.0);
    ndt_option_.remove_centroid_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/remove_centroid" ,false);
    ndt_option_.capacity_ = param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/capacity" ,100000);
    ndt_option_.nearby_type_ = static_cast<LocUtils::NdtNearbyType>(param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/nearby_type" ,1));
    ndt_option_.method_ = static_cast<LocUtils::NdtMethod>(param_ptr_->GetParamFromYaml("lio_mapping/ndt_option/method" ,1));

    icp_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/max_iteration" ,20);
    icp_option_.max_nn_distance_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/max_nn_distance" ,1.0);
    icp_option_.max_plane_distance_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/max_plane_distance" ,0.1);
    icp_option_.max_line_distance_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/max_line_distance" ,0.5);
    icp_option_.eps_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/eps" ,1e-2);
    icp_option_.euc_fitness_eps_ = param_ptr_->GetParamFromYaml("lio_mapping/icp_option/euc_fitness_eps" ,0.36);
    icp_option_.method_ = static_cast<LocUtils::IcpMethod>(param_ptr_->GetParamFromYaml("lio_mapping/icp_option/method" ,0));

    loam_option_.feature_option_.num_scan_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/num_scan" ,16);
    loam_option_.min_edge_pts_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/min_edge_pts" ,20);
    loam_option_.min_surf_pts_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/min_surf_pts" ,20);
    loam_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/max_iteration" ,20);
    loam_option_.use_edge_points_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/use_edge_points" ,true);
    loam_option_.use_surf_points_ = param_ptr_->GetParamFromYaml("lio_mapping/loam_option/use_surf_points" ,true);

    lio_option_.kf_distance_ = param_ptr_->GetParamFromYaml("lio_mapping/lio_option/kf_distance" ,0.5);
    lio_option_.kf_angle_deg_ = param_ptr_->GetParamFromYaml("lio_mapping/lio_option/kf_angle_deg" ,30);
    lio_option_.num_kfs_in_local_map_ = param_ptr_->GetParamFromYaml("lio_mapping/lio_option/num_kfs_in_local_map" ,10);

    lio_option_.gloal_map_filter_ = param_ptr_->GetParamFromYaml<double>("lio_mapping/lio_option/gloal_map_filter" ,0.5);
    lio_option_.local_map_filter_ = param_ptr_->GetParamFromYaml<double>("lio_mapping/lio_option/local_map_filter" ,0.5);
    lio_option_.cur_scan_filter_ = param_ptr_->GetParamFromYaml<double>("lio_mapping/lio_option/cur_scan_filter" ,1.0);
    
    lio_option_.key_frame_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/lio_option/key_frame_path" ,"/tmp/key_frames/");
    lio_option_.loam_key_frame_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/lio_option/loam_key_frame_path" ,"/tmp/key_frames_loam/");
    lio_option_.map_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/lio_option/map_path" ,"/tmp/map/");
    lio_option_.trajectory_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/lio_option/trajectory_path" ,"/tmp/trajectory/mapping/");
    
    //topic
    lidar_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/lidar_topic" ,"/rslidar_points");
    imu_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/imu_topic" ,"/gps/imu");
    gnss_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_mapping/gnss_topic" ,"/gps/fix");

    matching_method_ = static_cast<MatchingMethod>(param_ptr_->GetParamFromYaml("lio_mapping/matching_method" ,0));
    lidar_type_ = static_cast<LocUtils::LidarType>(param_ptr_->GetParamFromYaml("lio_mapping/lidar_type" ,0));
    
    if(matching_method_ != MatchingMethod::LOAM)
        lidar_type_ = LocUtils::LidarType::NOTYPE;
    
    b_use_imu_ = sync_option_.b_sync_imu_ = param_ptr_->GetParamFromYaml<bool>("lio_mapping/measure_sync/use_imu", false);
    b_use_gnss_ = sync_option_.b_sync_gnss_ = param_ptr_->GetParamFromYaml<bool>("lio_mapping/measure_sync/use_gnss", false);
    
    sync_option_.behind_time_limit_ = param_ptr_->GetParamFromYaml<double>("lio_mapping/measure_sync/behind_time_limit", 0.2);
    sync_option_.exced_time_limit_ = param_ptr_->GetParamFromYaml<double>("lio_mapping/measure_sync/exced_time_limit", 0.02);
    sync_option_.debug_info_ = param_ptr_->GetParamFromYaml<bool>("lio_mapping/measure_sync/debug_info", false);

    if(b_use_imu_)
    {
        lio_option_.ext_r = param_ptr_->GetParamFromYaml<std::vector<double>>("lio_mapping/lio_option/ext_r", std::vector<double>());
        lio_option_.ext_t = param_ptr_->GetParamFromYaml<std::vector<double>>("lio_mapping/lio_option/ext_t", std::vector<double>());
        b_use_eskf_ = param_ptr_->GetParamFromYaml<bool>("lio_mapping/use_eskf", false);
    }

    if(b_use_gnss_)
    {
        std::vector<double> ext_r_gnss_imu_v = 
            param_ptr_->GetParamFromYaml<std::vector<double>>("lio_mapping/ext_r_gnss_imu", std::vector<double>());
        std::vector<double> ext_t_gnss_imu_v = 
            param_ptr_->GetParamFromYaml<std::vector<double>>("lio_mapping/ext_t_gnss_imu", std::vector<double>());
  
        Vec3d T_gnss_imu = LocUtils::math::VecFromArray(ext_t_gnss_imu_v);
        Mat3d R_gnss_imu = LocUtils::math::RpyToRotM2(ext_r_gnss_imu_v[0], ext_r_gnss_imu_v[1], ext_r_gnss_imu_v[2]);

        T_gnss_imu_ = SE3(R_gnss_imu, T_gnss_imu);
    }

    LOG(INFO) << " \n" << 
               "matching type :" << static_cast<int>(matching_method_) << "\n"
               "lidar_type: " << static_cast<int>(lidar_type_);
    return true;
}

bool LioMappingFlow::InitSubPub()
{
    switch (matching_method_)
    {
        case MatchingMethod::LOAM:
            cloud_sub_ptr_ = std::make_shared<LocUtils::CloudSubscriber>(nh_, lidar_topic_, 1000, lidar_type_);

        break;
        default:
            cloud_sub_ptr_ = std::make_shared<LocUtils::CloudSubscriber>(nh_, lidar_topic_, 1000);
        break;
    }

    cur_scan_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_cur_scan_topic_, "map", 1000);
    local_map_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_local_map_topic_, "map", 1000);
    global_map_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_map_topic_, "map", 1000);

    cur_pos_pub_ptr_ = std::make_shared<LocUtils::OdometryPublisher>(nh_, k_cur_pos_topic_, "map", "velo_link", 1000);
    gnss_pos_pub_ptr_ = std::make_shared<LocUtils::OdometryPublisher>(nh_, k_cur_gnss_pos_topic_, "map", "gnss_link", 1000);

    if(b_use_imu_)
        imu_sub_ptr_ = std::make_shared<LocUtils::IMUSubscriber>(nh_, imu_topic_, 1000);
    if(b_use_gnss_)
        gnss_sub_ptr_ = std::make_shared<LocUtils::GnssSubscriber>(nh_, gnss_topic_, 1000);

    return true;
}

bool LioMappingFlow::InitLio()
{
    switch (matching_method_)
    {
        case MatchingMethod::LOAM:
            lio_ptr_ = std::make_shared<LocUtils::Lio>(lio_option_, loam_option_);
        break;
        case MatchingMethod::ICP:
            lio_ptr_ = std::make_shared<LocUtils::Lio>(lio_option_, icp_option_);
        break;
        case MatchingMethod::NDT:
            lio_ptr_ = std::make_shared<LocUtils::Lio>(lio_option_, ndt_option_);
        break;
        default:
            lio_ptr_ = std::make_shared<LocUtils::Lio>();
        break;
    }

    return true;
}

bool LioMappingFlow::InitSync()
{
    sync_ptr_ = std::make_shared<LocUtils::MappingMessageSync>(sync_option_);

    return true;
}

bool LioMappingFlow::Run()
{
    if(!InitStaticImu())
        return true;

    if(!ReadData())
        return false;

    while(HasData())
    {
        if(ValidData())
        { 
            if(b_use_gnss_)
            {
                TransData();
            }    

            if(!InitFirstPose())
                return false;

            if(UpdateMapping())
            {
                PublishData();
            }
        }
    }

    return true;
}

bool LioMappingFlow::InitFirstPose()
{
    static bool b_init_pos {false};
    if(!b_init_pos)
    {
        lio_ptr_->SetInitPose(init_pose_);
        b_init_pos = true;
    }

    return true;
}

bool LioMappingFlow::ReadData()
{
    if(b_use_imu_)
    {
        imu_sub_ptr_->ParseData(imu_data_buff_);
    }
        
    if(b_use_gnss_)
    {
        gnss_sub_ptr_->ParseData(gnss_data_buff_);   
    }

    if(matching_method_ == MatchingMethod::LOAM)
    {
        cloud_sub_ptr_->ParseData(full_cloud_data_buff_);
    }    
    else
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
    }
    return true;
}

bool LioMappingFlow::TransData()
{
    curr_gnss_ = sync_ptr_->GetMeasure().gnss_buff_.back();
    curr_gnss_.UpdateXYZ();
    
    gnss_pose_.translation()[0] = curr_gnss_.local_E_;
    gnss_pose_.translation()[1] = curr_gnss_.local_N_;
    gnss_pose_.translation()[2] = curr_gnss_.local_U_;
    if(b_use_imu_)
    {
        curr_imu_ = sync_ptr_->GetMeasure().imu_buff_.back();
        // LOG(INFO) << curr_imu_.ori.toRotationMatrix();
        gnss_pose_.setRotationMatrix(T_gnss_imu_.rotationMatrix() * curr_imu_.ori.toRotationMatrix());
        init_pose_ = gnss_pose_;
    }
    
    allframe_gnss_pose_buff_.emplace_back(gnss_pose_);
    return true;
}

bool LioMappingFlow::HasData()
{
    if(!(matching_method_ == MatchingMethod::LOAM) && cloud_data_buff_.size() == 0)
        return false;
    if(matching_method_ == MatchingMethod::LOAM && full_cloud_data_buff_.size() == 0)
        return false;
    if(b_use_imu_ && imu_data_buff_.size() == 0)
        return false;
    if(b_use_gnss_ && gnss_data_buff_.size() == 0)
        return false;
    return true;
}

bool LioMappingFlow::ValidData()
{

    if(matching_method_ == MatchingMethod::LOAM)
    {
        curr_full_scan_ = full_cloud_data_buff_.front();
        sync_ptr_->ProcessCloud(curr_full_scan_);
    }
    else
    {
        curr_scan_ = cloud_data_buff_.front();
        sync_ptr_->ProcessCloud(curr_scan_);
    }

    if(b_use_imu_)
    {
        sync_ptr_->ProcessImu(imu_data_buff_);
    }
    
    if(b_use_gnss_)
    {
        sync_ptr_->ProcessGnss(gnss_data_buff_);
    }

    if(matching_method_ == MatchingMethod::LOAM)
        full_cloud_data_buff_.pop_front();
    else
        cloud_data_buff_.pop_front();

    if(!sync_ptr_->IsSync())
    {
        return false;
    }

    // 时间同步后 获取与最新cloud最近的 gnss
    if(!InitGNSS())
        return false;

    return true;
}

bool LioMappingFlow::UpdateMapping()
{
    //只用点云做建图
    if(!b_use_eskf_)
    {
        if(matching_method_ == MatchingMethod::LOAM)
            lio_ptr_->AddCloud(sync_ptr_->GetMeasure().lidar_full_, cur_pose_);
        else
            lio_ptr_->AddCloud(sync_ptr_->GetMeasure().lidar_normal_, cur_pose_);
        return true;
    }

    lio_ptr_->AddMeasure(sync_ptr_->GetMeasure(), cur_pose_);
    
    return true;
}

bool LioMappingFlow::PublishData()
{
    double cloud_time;

    if(matching_method_ == MatchingMethod::LOAM)
    {
        cloud_time = curr_full_scan_.time;
    }
    else
    {
        cloud_time = curr_scan_.time;
    }

    LocUtils::CloudPtr pub_cur_scan(new LocUtils::PointCloudType);
    LocUtils::CloudPtr pub_local_map(new LocUtils::PointCloudType);
    
    if(lio_ptr_->GetCurrentScan(pub_cur_scan))
        cur_scan_pub_ptr_->Publish(pub_cur_scan);

    if(lio_ptr_->GetLocalMap(pub_local_map))
        local_map_pub_ptr_->Publish(pub_local_map);
    
    if(b_use_gnss_)
        gnss_pos_pub_ptr_->Publish(gnss_pose_, cloud_time);

    cur_pos_pub_ptr_->Publish(cur_pose_, cloud_time);
    return true;
}

bool LioMappingFlow::SaveGlobalMap()
{
    LocUtils::CloudPtr pub_global_map(new LocUtils::PointCloudType);

    if(!lio_ptr_->SaveGlobalMap("map", pub_global_map))
    {
        return false;
    }
    
    global_map_pub_ptr_->Publish(pub_global_map);
    return true;
}

bool LioMappingFlow::SaveAllKeyframePoses()
{
    std::shared_ptr<std::deque<SE3>> all_keyframes_ptr
        = std::make_shared<std::deque<SE3>>();
    if(!lio_ptr_->GetAllKeyFramePose(all_keyframes_ptr))
    {
        return false;
    }
    std::vector<double> temp_times(1, 0.0);//临时给个time 因为没保存
    // 只能保存KITTI格式！！！！
    LocUtils::SavePose save_pose(LocUtils::SavePose::SavePoseMethod::KITTI, lio_option_.trajectory_path_ + "mapping.txt");
    save_pose.SaveAllPoseToFile(*all_keyframes_ptr, temp_times);
    return true;
}

//因为Lio的写法，因此gnss暂时无法跟lio关键帧一一对应起来，暂时只保存所有gnss
bool LioMappingFlow::SaveAllGnssPoses()
{
    if(allframe_gnss_pose_buff_.empty())
    {
        return false;
    }
    LocUtils::SavePose save_pose(LocUtils::SavePose::SavePoseMethod::KITTI, lio_option_.trajectory_path_ + "gnss.txt");
    std::vector<double> temp_times(1, 0.0);//临时给个time 因为没保存
    // 只能保存KITTI格式！！！！
    save_pose.SaveAllPoseToFile(allframe_gnss_pose_buff_, temp_times);
    return true;
}