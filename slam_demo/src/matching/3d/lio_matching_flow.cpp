//
// Created by Tian on 2023/10/27.
//

#include "matching/3d/lio_matching_flow.hpp"

LioMatchingFlow::LioMatchingFlow(ros::NodeHandle& nh, std::string yaml_path)
                              :nh_(nh)
{
    param_ptr_ = std::make_shared<LocUtils::Parameters>(yaml_path);
    InitConfig();
    InitSubPub();
    InitLoc();
}

bool LioMatchingFlow::InitConfig()
{
    ndt_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/max_iteration" ,20);
    ndt_option_.voxel_size_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/voxel_size" ,1.0);
    ndt_option_.inv_voxel_size_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/inv_voxel_size" ,1.0);
    ndt_option_.min_effective_pts_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/min_effective_pts" ,10);
    ndt_option_.min_pts_in_voxel_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/min_pts_in_voxel" ,3);
    ndt_option_.max_pts_in_voxel_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/max_pts_in_voxel" ,50);
    ndt_option_.eps_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/eps" ,1e-2);
    ndt_option_.res_outlier_th_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/res_outlier_th" ,20.0);
    ndt_option_.remove_centroid_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/remove_centroid" ,false);
    ndt_option_.capacity_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/capacity" ,100000);
    ndt_option_.capacity_ = param_ptr_->GetParamFromYaml("lio_matching/ndt_option/capacity" ,100000);
    ndt_option_.nearby_type_ = static_cast<LocUtils::NdtNearbyType>(param_ptr_->GetParamFromYaml("lio_matching/ndt_option/nearby_type" ,1));
    ndt_option_.method_ = static_cast<LocUtils::NdtMethod>(param_ptr_->GetParamFromYaml("lio_matching/ndt_option/method" ,1));

    icp_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/max_iteration" ,20);
    icp_option_.max_nn_distance_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/max_nn_distance" ,1.0);
    icp_option_.max_plane_distance_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/max_plane_distance" ,0.1);
    icp_option_.max_line_distance_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/max_line_distance" ,0.5);
    icp_option_.eps_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/eps" ,1e-2);
    icp_option_.euc_fitness_eps_ = param_ptr_->GetParamFromYaml("lio_matching/icp_option/euc_fitness_eps" ,0.36);
    icp_option_.method_ = static_cast<LocUtils::IcpMethod>(param_ptr_->GetParamFromYaml("lio_matching/icp_option/method" ,0));

    loam_option_.feature_option_.num_scan_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/num_scan" ,16);
    loam_option_.min_edge_pts_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/min_edge_pts" ,20);
    loam_option_.min_surf_pts_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/min_surf_pts" ,20);
    loam_option_.max_iteration_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/max_iteration" ,20);
    loam_option_.use_edge_points_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/use_edge_points" ,true);
    loam_option_.use_surf_points_ = param_ptr_->GetParamFromYaml("lio_matching/loam_option/use_surf_points" ,true);
    
    loc_option_.cur_scan_filter_ = param_ptr_->GetParamFromYaml<double>("lio_matching/loc_option/cur_scan_filter" ,1.0);
    loc_option_.global_map_filter_ = param_ptr_->GetParamFromYaml<double>("lio_matching/loc_option/global_map_filter" ,0.5);
    loc_option_.box_filter_size_ = param_ptr_->GetParamFromYaml<std::vector<float>>("lio_matching/loc_option/box_filter_size" ,std::vector<float>());
    loc_option_.loam_edge_map_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/loc_option/edge_loam_map_path" ,"/tmp/map/edge_map.pcd");
    loc_option_.loam_surf_map_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/loc_option/surf_loam_map_path" ,"/tmp/map/surf_map.pcd");
    loc_option_.map_path_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/loc_option/map_path" ,"/tmp/map/map.pcd");
    
    lidar_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/lidar_topic" ,"/rslidar_points");
    imu_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/imu_topic" ,"/imu_data"); 
    gnss_topic_ = param_ptr_->GetParamFromYaml<std::string>("lio_matching/gnss_topic" ,"/fix"); 

    b_use_imu_ = sync_option_.b_sync_imu_ = param_ptr_->GetParamFromYaml<bool>("lio_matching/use_imu" , false); 
    b_use_gnss_ = sync_option_.b_sync_gnss_ = param_ptr_->GetParamFromYaml<bool>("lio_matching/use_gnss" , false); 
    
    if (b_use_gnss_)
    {
        LocUtils::GnssData::origin_latitude = param_ptr_->GetParamFromYaml<double>("lio_matching/origin_latitude" , 36.509905); 
        LocUtils::GnssData::origin_longitude = param_ptr_->GetParamFromYaml<double>("lio_matching/origin_longitude" , 116.802690); 
        LocUtils::GnssData::origin_altitude = param_ptr_->GetParamFromYaml<double>("lio_matching/origin_altitude" , 69.921000); 

        std::vector<double> ext_r_gnss_imu_v = 
            param_ptr_->GetParamFromYaml<std::vector<double>>("lio_matching/ext_r_gnss_imu", std::vector<double>());
        std::vector<double> ext_t_gnss_imu_v = 
            param_ptr_->GetParamFromYaml<std::vector<double>>("lio_matching/ext_t_gnss_imu", std::vector<double>());
  
        Vec3d T_gnss_imu = LocUtils::math::VecFromArray(ext_t_gnss_imu_v);
        Mat3d R_gnss_imu = LocUtils::math::RpyToRotM2(ext_r_gnss_imu_v[0], ext_r_gnss_imu_v[1], ext_r_gnss_imu_v[2]);

        T_gnss_imu_ = SE3(R_gnss_imu, T_gnss_imu);
    }

    if (b_use_imu_)
    {
        loc_option_.ext_r = param_ptr_->GetParamFromYaml<std::vector<double>>("lio_matching/loc_option/ext_r", std::vector<double>());
        loc_option_.ext_t = param_ptr_->GetParamFromYaml<std::vector<double>>("lio_matching/loc_option/ext_t", std::vector<double>());
        b_use_eskf_ = param_ptr_->GetParamFromYaml<bool>("lio_matching/use_eskf" , false); 
    }

    matching_method_ = static_cast<MatchingMethod>(param_ptr_->GetParamFromYaml("lio_matching/matching_method" ,0));
    lidar_type_ = static_cast<LocUtils::LidarType>(param_ptr_->GetParamFromYaml("lio_matching/lidar_type" ,0));
}

bool LioMatchingFlow::Run()
{
    if(
        loc_ptr_->HasNewGlobalMap() && 
        global_map_pub_ptr_->HasSubscribers())
    {
        global_map_pub_ptr_->Publish(loc_ptr_->GetGlobalMap());
    }

    if(
        loc_ptr_->HasNewLocalMap() && 
        local_map_pub_ptr_->HasSubscribers()
    )
    {
        LOG(INFO) << " Pub local data";
        local_map_pub_ptr_->Publish(loc_ptr_->GetLocalMap());
    }

    //step 1 初始化imu的ba bg （如果使用imu的话）
    if(!InitStaticImu())
        return false;
    //step 2 读取数据
    if(!ReadData())
        return false;
    //step 3 初始化定位姿态
    if(!InitPosition())
        return false;
    
    static double lidar_time,imu_time, gnss_time;
    if(!HasGnssData()
        && !HasImuData()
        && !HasLidarData())
        return false;

    if(HasLidarData())
    {
        if(matching_method_ == MatchingMethod::LOAM)
        {
            curr_full_scan_ = full_cloud_data_buff_.front();
            full_cloud_data_buff_.pop_front();
            loc_ptr_->Update(curr_full_scan_);
            
        }
        else
        {
            curr_scan_ = cloud_data_buff_.front();
            // lidar_time = curr_scan_.time;
            cloud_data_buff_.pop_front();
            loc_ptr_->Update(curr_scan_);
        }
    }

    if(HasImuData())
    {
        curr_imu_ = imu_data_buff_.front();
        imu_data_buff_.pop_front();
        imu_time = curr_imu_.timestamp_;
        loc_ptr_->Update(curr_imu_);

        // double diff_lidar_imu = lidar_time - imu_time;
        // LOG(INFO) << "diff_lidar_imu " << diff_lidar_imu;
    }
        
    if(HasGnssData())
    {
        // gnss_time
        gnss_data_buff_.pop_front();
    }

    cur_pos_ = loc_ptr_->GetCurrPos();
    // LOG(INFO) << cur_pos_.matrix();
    PublishData();
    return true;
}

bool LioMatchingFlow::ReadData()
{

    if(matching_method_ == MatchingMethod::LOAM)    
        cloud_sub_ptr_->ParseData(full_cloud_data_buff_);
    else
        cloud_sub_ptr_->ParseData(cloud_data_buff_);

    
    if(b_use_imu_)
        imu_sub_ptr_->ParseData(imu_data_buff_);
    
    if(b_use_gnss_)
        gnss_sub_ptr_->ParseData(gnss_data_buff_);
        
    return true;
}

bool LioMatchingFlow::InitStaticImu()
{
    // 没使用 imu 或者不使用eskf就不需要initstatic imu
    if(!b_use_imu_ || !b_use_eskf_)
    {
        return true;
    }

    if(b_static_imu_)
        return true;
        
    imu_sub_ptr_->ParseData(imu_data_buff_);
    b_static_imu_ = loc_ptr_->InitImu(imu_data_buff_);
    return true;
}

bool LioMatchingFlow::InitPosition()
{
    if(loc_ptr_->HasInit())
        return true;

    if(!b_use_gnss_)
    {
        loc_ptr_->SetInitPose(SE3());
        return true;
    }

    while(HasLidarData()
        && HasImuData()
        && HasGnssData())
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
        sync_ptr_->ProcessImu(imu_data_buff_);         
        sync_ptr_->ProcessGnss(gnss_data_buff_);

        // gps刚开始信号不一定好 如果使用imu正好开始初始化imu 默认其实2s是静止
        static int gnss_cnt = 0;
        if(gnss_cnt > 3)
        {
            if(sync_ptr_->IsSync())
            {
                TransData();
                LOG(INFO) << init_pos_.matrix();
                loc_ptr_->SetInitPose(init_pos_);
                // sync_ptr_.reset();
                return true;
            }    
        }

        if(matching_method_ == MatchingMethod::LOAM)
            full_cloud_data_buff_.pop_front();
        else
            cloud_data_buff_.pop_front();
        gnss_cnt++;
    }
    return false;
}

void LioMatchingFlow::TransData()
{
    curr_gnss_ = sync_ptr_->GetMeasure().gnss_buff_.back();
    // LOG(INFO) << curr_gnss_.origin_altitude;
    curr_gnss_.UpdateXYZ();
    init_pos_.translation()[0] = curr_gnss_.local_E_;
    init_pos_.translation()[1] = curr_gnss_.local_N_;
    init_pos_.translation()[2] = curr_gnss_.local_U_;

    if(b_use_imu_)
    {
        curr_imu_ = sync_ptr_->GetMeasure().imu_buff_.back();
        // LOG(INFO) << curr_imu_.ori.toRotationMatrix();
        init_pos_.setRotationMatrix(T_gnss_imu_.rotationMatrix() * curr_imu_.ori.toRotationMatrix());
    }
}

bool LioMatchingFlow::InitSubPub()
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

    if(b_use_imu_)
        imu_sub_ptr_ = std::make_shared<LocUtils::IMUSubscriber>(nh_, imu_topic_, 1000);

    if(b_use_gnss_)
        gnss_sub_ptr_ = std::make_shared<LocUtils::GnssSubscriber>(nh_, gnss_topic_, 1000);

    cur_scan_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_cur_scan_topic_, "map", 1000);
    global_map_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_map_topic_, "map", 1000);
    local_map_pub_ptr_ = std::make_shared<LocUtils::CloudPublisher>(nh_, k_local_map_topic_, "map", 1000);
    cur_pos_pub_ptr_ = std::make_shared<LocUtils::OdometryPublisher>(nh_, k_cur_pos_topic_, "map", "velo_link", 1000);

    return true;
}

bool LioMatchingFlow::InitLoc()
{
    switch (matching_method_)
    {
        case MatchingMethod::LOAM:
            loc_ptr_ = std::make_shared<LocUtils::Loc>(loc_option_, loam_option_);
        break;
        case MatchingMethod::ICP:
            loc_ptr_ = std::make_shared<LocUtils::Loc>(loc_option_, icp_option_);
        break;
        case MatchingMethod::NDT:
            loc_ptr_ = std::make_shared<LocUtils::Loc>(loc_option_, ndt_option_);
        break;
        default:
            loc_ptr_ = std::make_shared<LocUtils::Loc>();
        break;
    }

    sync_ptr_ = std::make_shared<LocUtils::MappingMessageSync>(sync_option_);
    
    return true;
}

bool LioMatchingFlow::HasLidarData()
{
    if(!(matching_method_ == MatchingMethod::LOAM) && !cloud_data_buff_.empty())
        return true;
    if(matching_method_ == MatchingMethod::LOAM && !full_cloud_data_buff_.empty())
        return true;
    
    
    return false;
}

bool LioMatchingFlow::HasImuData()
{
    if(b_use_imu_ && imu_data_buff_.empty())
        return false;
        
    return true;
}

bool LioMatchingFlow::HasGnssData()
{
    if(b_use_gnss_ && gnss_data_buff_.empty())
        return false;
    
    static bool init_gnss = false;
    if(!init_gnss)
    {
        curr_gnss_ = gnss_data_buff_.front();
        curr_gnss_.InitManualPosition();
        init_gnss = true;
    }
    return true;
}

bool LioMatchingFlow::ValidData()
{
    if(matching_method_ == MatchingMethod::LOAM)
        curr_full_scan_ = full_cloud_data_buff_.front();
    else
        curr_scan_ = cloud_data_buff_.front();
    
    if(matching_method_ == MatchingMethod::LOAM)
        full_cloud_data_buff_.pop_front();
    else
        cloud_data_buff_.pop_front();

    return true;
}

bool LioMatchingFlow::UpdateMatching()
{
    if(!loc_ptr_->HasInit())
    {
        //暂时以第一帧为原点去做
        LOG(INFO) << "set init pos " << init_pos_.matrix();
        loc_ptr_->SetInitPose(init_pos_);
        return false;
    }

    // if(matching_method_ == MatchingMethod::LOAM)
    //     loc_ptr_->Update(curr_full_scan_);
    // else
    //     loc_ptr_->Update(curr_scan_);

    return true;
}

bool LioMatchingFlow::PublishData()
{
    cur_pos_pub_ptr_->Publish(cur_pos_);

    LocUtils::CloudPtr pub_cur_scan(new LocUtils::PointCloudType);
    if(loc_ptr_->GetCurrentScan(pub_cur_scan))
        cur_scan_pub_ptr_->Publish(pub_cur_scan);
    // cur_scan_pub_ptr_->Publish()
    return true;
}
