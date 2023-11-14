//
// Created by Tian on 2023/09/18.
//

#include "LocUtils/slam/3d/lio.hpp"

namespace LocUtils
{
    Lio::Lio()
    {
        match_ptr_ = std::make_shared<IcpRegistration>();
        InitEskf();
        InitDataPath();
        InitCloudFilter();
    }

    Lio::Lio(LioOptions lio_option)
        : lio_option_(lio_option)
    {
        match_ptr_ = std::make_shared<IcpRegistration>();
        InitEskf();
        InitDataPath();
        InitCloudFilter();        
    }

    Lio::Lio(LioOptions lio_option, NdtOptions ndt_option)
        : lio_option_(lio_option)
          ,ndt_option_(ndt_option)
    {
        match_ptr_ = std::make_shared<NdtRegistration>(ndt_option);
        InitEskf();
        InitDataPath();
        InitCloudFilter();        
    }
    
    Lio::Lio(LioOptions lio_option, IcpOptions icp_option)
        : lio_option_(lio_option)
         ,icp_option_(icp_option)
    {
        match_ptr_ = std::make_shared<IcpRegistration>(icp_option);
        InitEskf();
        InitDataPath();
        InitCloudFilter();        
    }
    
    Lio::Lio(LioOptions lio_option, LoamOption loam_option)
        : lio_option_(lio_option)
          ,loam_option_(loam_option)
    {
        b_use_loam_ = true;
        match_ptr_ = std::make_shared<LoamRegistration>(loam_option_);
        loam_feature_ptr_ = std::make_shared<LoamFeatureExtract>(loam_option_.feature_option_);
        InitEskf();
        InitDataPath();
        InitCloudFilter();        
    }

    void Lio::InitEskf()
    {
        static_imu_init_ptr_ = std::make_shared<LocUtils::StaticIMUInit>();
        eskf_ptr_ = std::make_shared<LocUtils::ESKFD>();

        Vec3d T_imu_lidar = math::VecFromArray(lio_option_.ext_t);
        Mat3d R_imu_lidar = math::RpyToRotM2(lio_option_.ext_r[0], lio_option_.ext_r[1], lio_option_.ext_r[2]);

        TIL_ = SE3(R_imu_lidar, T_imu_lidar);
        LOG(INFO) << "---------- TIL_ ---------" << TIL_.matrix();
    }

    bool Lio::InitImu(std::deque<IMU> &imu_buff)
    {
        if(static_imu_init_ptr_ == nullptr)
            static_imu_init_ptr_ = std::make_shared<LocUtils::StaticIMUInit>();
        
        if(true == static_imu_init_ptr_->InitSuccess())
        {
            LOG(INFO) << " static imu init ok ";
            return true;
        }

        while (!imu_buff.empty())
        {
            IMU cur_imu_data = imu_buff.front();
            static_imu_init_ptr_->AddIMU(cur_imu_data);
            if(true == static_imu_init_ptr_->InitSuccess())
            {
                LOG(INFO) << " static imu init ok ";
                EskfOptions eskf_option;
                eskf_option.gyro_var_ = sqrt(static_imu_init_ptr_->GetCovGyro()[0]);
                eskf_option.acce_var_ = sqrt(static_imu_init_ptr_->GetCovAcce()[0]);
                eskf_ptr_->SetInitialConditions(eskf_option, static_imu_init_ptr_->GetInitBg(),
                                                        static_imu_init_ptr_->GetInitBa(), static_imu_init_ptr_->GetGravity());
                has_init_eskf_ = true;
                return true;
            } 
            imu_buff.pop_front();
        }
        return static_imu_init_ptr_->InitSuccess();
    }

    void Lio::InitDataPath()
    {
        FileManager::InitDirectory(lio_option_.key_frame_path_, "Point Cloud Key Frames");
        FileManager::InitDirectory(lio_option_.loam_key_frame_path_, "Loam Point Cloud Key Frames");
        FileManager::InitDirectory(lio_option_.map_path_, "Map Save");
        FileManager::InitDirectory(lio_option_.trajectory_path_, "Estimated Trajectory");
    }

    void Lio::InitCloudFilter()
    {
        cur_scan_filter_ptr_ = std::make_shared<VoxelFilter>(lio_option_.cur_scan_filter_);
        local_map_filter_ptr_ = std::make_shared<VoxelFilter>(lio_option_.local_map_filter_);
        global_map_filter_ptr_ = std::make_shared<VoxelFilter>(lio_option_.gloal_map_filter_);
    }

    void Lio::SaveKeyFrame(CloudPtr scan, const size_t& idx)
    {
        std::string file_path = lio_option_.key_frame_path_ + std::to_string(idx) + ".pcd";
        SaveCloudToFile(file_path, scan); 
    }

    void Lio::SaveKeyFrame(CloudPtr edge_scan, CloudPtr surf_scan, const size_t& idx)
    {
        std::string edge_file_path = lio_option_.loam_key_frame_path_ + "edge_" + std::to_string(idx) + ".pcd";
        std::string surf_file_path = lio_option_.loam_key_frame_path_ + "surf_" + std::to_string(idx) + ".pcd";
        
        SaveCloudToFile(edge_file_path, edge_scan); 
        SaveCloudToFile(surf_file_path, surf_scan); 
    }

    bool Lio::SaveGlobalMap(const std::string file_name)
    {
        if(b_use_loam_)
        {
            std::string edge_map_path = lio_option_.map_path_ + "edge_" + file_name + ".pcd";
            std::string surf_map_path = lio_option_.map_path_ + "surf_" + file_name + ".pcd";;
            std::string map_path = lio_option_.map_path_ + file_name + ".pcd";
            CloudPtr edge_map (new PointCloudType), surf_map (new PointCloudType), global_map (new PointCloudType);
            if(!GetGlobalMap(edge_map, surf_map))
            { 
                LOG(INFO) << "Get Global Map Failed!";
                return false;
            }
            *global_map = *edge_map;
            *global_map += *surf_map;

            global_map_filter_ptr_->Filter(global_map, global_map);

            SaveCloudToFile(edge_map_path, edge_map);
            SaveCloudToFile(surf_map_path, surf_map);
            SaveCloudToFile(map_path, global_map);

            LOG(INFO) << "LOAM Global Map be Saved to " << lio_option_.map_path_ << " Successfully.";
        }
        else
        {
            std::string file_path = lio_option_.map_path_ + file_name + ".pcd";
            CloudPtr global_map(new PointCloudType);
            if(!GetGlobalMap(global_map))
            {
                LOG(INFO) << "Get Global Map Failed!";
                return false;
            }

            SaveCloudToFile(file_path, global_map);
            LOG(INFO) << "Global Map be Saved to " << lio_option_.map_path_ << " Successfully.";
            global_map = nullptr;   //手动释放，其实程序应该可以自动释放。
        }
        return true;
    }

    bool Lio::SaveGlobalMap(const std::string file_name, CloudPtr &global_map)
    {
        if(b_use_loam_)
        {
            std::string edge_map_path = lio_option_.map_path_ + "edge_" + file_name + ".pcd";
            std::string surf_map_path = lio_option_.map_path_ + "surf_" + file_name + ".pcd";;
            std::string map_path = lio_option_.map_path_ + file_name + ".pcd";
            CloudPtr edge_map (new PointCloudType), surf_map (new PointCloudType);
            if(!GetGlobalMap(edge_map, surf_map))
            { 
                LOG(INFO) << "Get Global Map Failed!";
                return false;
            }
            *global_map = *edge_map;
            *global_map += *surf_map;

            SaveCloudToFile(edge_map_path, edge_map);
            SaveCloudToFile(surf_map_path, surf_map);
            SaveCloudToFile(map_path, global_map);

            LOG(INFO) << "LOAM Global Map be Saved to " << lio_option_.map_path_ << " Successfully.";
        }
        else
        {
            std::string file_path = lio_option_.map_path_ + file_name + ".pcd";
            if(!GetGlobalMap(global_map))
            {
                LOG(INFO) << "Get Global Map Failed!";
                return false;
            }

            SaveCloudToFile(file_path, global_map);
            LOG(INFO) << "Global Map be Saved to " << lio_option_.map_path_ << " Successfully.";
        }
        return true;
    }

    void Lio::SetInitPose(const SE3 &pose)
    {
        if(estimated_poses_.empty())
        {
            last_kf_pose_ = pose;
            estimated_poses_.push_back(last_kf_pose_);
        }
    }

    //直接法的lio
    void Lio::AddCloud(CloudPtr scan, SE3& pose)
    {
        static bool start_lidar_lio {false};

        if(!start_lidar_lio)
        {
            LOG(INFO) << " start normal lio ------------\n";
            start_lidar_lio = true;
        }

        if(b_use_loam_)
        {
            LOG(INFO) << " cloud type input err using loam FullCloudType ";
            return;
        }
        
        CloudPtr source_scan (new PointCloudType);
        cur_scan_filter_ptr_->Filter(scan, source_scan);
        // LOG(INFO) << " source_scan SIZE " << source_scan->points.size();
        if (local_map_ == nullptr)
        {
            // 第一个帧，直接加入local map
            local_map_.reset(new PointCloudType);
            CloudPtr key_frame_scan(new PointCloudType);
            pcl::transformPointCloud(*source_scan, *key_frame_scan, last_kf_pose_.matrix());
            *local_map_ += *key_frame_scan;
            
            pose = last_kf_pose_;
            
            local_map_filter_ptr_->Filter(local_map_, local_map_);

            match_ptr_->SetInputTarget(local_map_);

            scans_in_local_map_.push_back(key_frame_scan);

            SaveKeyFrame(scan, estimated_poses_.size());
            return;
        }   
           
        pose = AlignWithLocalMap(source_scan);

        if(has_init_eskf_)
        {
            T_w_i_eskf_ = pose * TIL_.inverse();
            eskf_ptr_->ObserveSE3(T_w_i_eskf_);
            pose = eskf_ptr_->GetNominalSE3() * TIL_;
        }
        // LOG(INFO) << " caculate pos";
        //是不是关键帧
        if(IsKeyframe(pose))
        {
            last_kf_pose_ = pose;

            // 重建local map
            estimated_poses_.push_back(last_kf_pose_);
            
            SaveKeyFrame(scan, estimated_poses_.size());

            CloudPtr key_frame_scan (new PointCloudType);
            pcl::transformPointCloud(*scan, *key_frame_scan, pose.matrix());

            scans_in_local_map_.push_back(key_frame_scan);

            if(scans_in_local_map_.size() > lio_option_.num_kfs_in_local_map_)
            {
                scans_in_local_map_.pop_front();
                local_map_.reset(new PointCloudType);
                
                for (auto& sc : scans_in_local_map_) 
                {
                    *local_map_ += *sc;
                }
            }
            else
            {
                *local_map_ += *key_frame_scan;
            }

            local_map_filter_ptr_->Filter(local_map_, local_map_);
            if(ndt_option_.method_ == NdtMethod::INCREMENTAL_NDT)
            {
                match_ptr_->SetInputTarget(key_frame_scan);
            }
            else
            {
                match_ptr_->SetInputTarget(local_map_);
            }
            
        }
    }

    //loam系的lio
    void Lio::AddCloud(FullCloudPtr scan, SE3& pose)
    {
        static bool start_lidar_lio {false};

        if(!start_lidar_lio)
        {
            LOG(INFO) << " start normal lio ------------\n";
            start_lidar_lio = true;
        }
        // pcl::transformPointCloud();
        CloudPtr edge_cloud(new PointCloudType);
        CloudPtr surf_cloud(new PointCloudType);
        loam_feature_ptr_->Extract(scan, edge_cloud, surf_cloud);

        if (edge_cloud->size() < loam_option_.min_edge_pts_ || surf_cloud->size() < loam_option_.min_surf_pts_) 
        {
            LOG(ERROR) << "not enough edge/surf pts: " << edge_cloud->size() << "," << surf_cloud->size();
            return;
        }

        if(local_map_edge_ == nullptr || local_map_surf_ == nullptr)
        {
            local_map_edge_.reset(new PointCloudType);
            local_map_surf_.reset(new PointCloudType);
            // 首帧特殊处理
            pose = last_kf_pose_;
            
            local_map_filter_ptr_->Filter(local_map_edge_, local_map_edge_);
            local_map_filter_ptr_->Filter(local_map_surf_, local_map_surf_);

            CloudPtr edge_key_frame_scan (new PointCloudType);
            CloudPtr surf_key_frame_scan (new PointCloudType);
            pcl::transformPointCloud(*edge_cloud, *edge_key_frame_scan, last_kf_pose_.matrix());
            pcl::transformPointCloud(*surf_cloud, *surf_key_frame_scan, last_kf_pose_.matrix());

            match_ptr_->SetInputTarget(edge_key_frame_scan, surf_key_frame_scan);
            
            *local_map_edge_ = *edge_key_frame_scan;
            *local_map_surf_ = *surf_key_frame_scan;

            edge_scans_in_local_map_.push_back(edge_key_frame_scan);
            surf_scans_in_local_map_.push_back(surf_key_frame_scan);

            SaveKeyFrame(edge_cloud, surf_cloud, estimated_poses_.size());

            return;
        }

        pose = AlignWithLocalMap(edge_cloud, surf_cloud);

        if(has_init_eskf_)
        {
            T_w_i_eskf_ = pose * TIL_.inverse();
            eskf_ptr_->ObserveSE3(T_w_i_eskf_);
            pose = eskf_ptr_->GetNominalSE3() * TIL_;
        }
        //是不是关键帧
        if(IsKeyframe(pose))
        {
            last_kf_pose_ = pose;

            // 重建local map
            estimated_poses_.push_back(last_kf_pose_);
            
            SaveKeyFrame(edge_cloud, surf_cloud, estimated_poses_.size());

            CloudPtr edge_key_frame_scan (new PointCloudType);
            CloudPtr surf_key_frame_scan (new PointCloudType);
            pcl::transformPointCloud(*edge_cloud, *edge_key_frame_scan, pose.matrix());
            pcl::transformPointCloud(*surf_cloud, *surf_key_frame_scan, pose.matrix());

            edge_scans_in_local_map_.push_back(edge_key_frame_scan);
            surf_scans_in_local_map_.push_back(surf_key_frame_scan);
            
            if(edge_scans_in_local_map_.size() > lio_option_.num_kfs_in_local_map_)
            {
                edge_scans_in_local_map_.pop_front();
                surf_scans_in_local_map_.pop_front();

                local_map_surf_.reset(new PointCloudType);
                local_map_edge_.reset(new PointCloudType);

                for(int i {0}; i < edge_scans_in_local_map_.size(); ++i)
                {
                    *local_map_surf_ += *surf_scans_in_local_map_.at(i);
                    *local_map_edge_ += *edge_scans_in_local_map_.at(i);
                }
            }
            else
            {
                *local_map_surf_ += *surf_key_frame_scan;
                *local_map_edge_ += *edge_key_frame_scan;
            }

            local_map_filter_ptr_->Filter(local_map_edge_, local_map_edge_);
            local_map_filter_ptr_->Filter(local_map_surf_, local_map_surf_);

            match_ptr_->SetInputTarget(local_map_edge_, local_map_surf_); 
        }
    }   

    void Lio::AddMeasure(const MappingMeasureGroup &measure, SE3& pose)
    {
        static bool start_eskf_lio {false};

        if(!start_eskf_lio)
        {
            LOG(INFO) << " start eskf lio ------------\n";

            // T_imu_baselink * T_w_lidar * T_lidar_imu  
            T_w_i_eskf_ = last_kf_pose_ * TIL_.inverse();
            Vec3d g_w_i = T_w_i_eskf_ * eskf_ptr_->GetGravity();
            LOG(INFO) << "g_w_i : " << g_w_i;
            eskf_ptr_->SetX(T_w_i_eskf_, g_w_i);
            start_eskf_lio = true;
        }

        for(auto &imu_data: measure.imu_buff_)
        {
            eskf_ptr_->Predict(imu_data);
        }
        //T_w_i
        T_w_i_eskf_ = eskf_ptr_->GetNominalSE3();

        // // LOG(INFO) << T_w_i_eskf_.matrix();
        if(b_use_loam_)
        {
            AddCloud(measure.lidar_full_, pose);
        }
        else
        {
            AddCloud(measure.lidar_normal_, pose);
        }
    }

    SE3 Lio::AlignWithLocalMap(CloudPtr source_cloud)
    {
        static SE3 last_pose = SE3();
        static SE3 predict_pos = SE3();
        
        current_scan_.reset(new PointCloudType);
        SE3 result_pos;
        source_cloud = RemoveNanPoint(source_cloud);
        if(has_init_eskf_)
        {
            match_ptr_->ScanMatch(source_cloud, T_w_i_eskf_ * TIL_, current_scan_, result_pos);
        }
        else
        {
            match_ptr_->ScanMatch(source_cloud, predict_pos, current_scan_, result_pos);
        }
        
        // predict_pos = result_pos;
        
        predict_pos = result_pos * last_pose.inverse() * result_pos;
        // predict_pos.so3().normalize();
        last_pose = result_pos;

        // LOG(INFO) << "result_pos: " << result_pos.translation().transpose() << ", "
        //       << result_pos.so3().unit_quaternion().coeffs().transpose();

        return result_pos;
    }

    SE3 Lio::AlignWithLocalMap(CloudPtr edge_cloud, CloudPtr surf_cloud)
    {
        static SE3 last_pose = SE3();
        static SE3 predict_pos = SE3();

        current_scan_.reset(new PointCloudType);
        SE3 result_pos;
        // source_cloud = RemoveNanPoint(source_cloud);
        // edge_cloud = RemoveNanPoint(edge_cloud);
        // surf_cloud = RemoveNanPoint(surf_cloud);
        cur_scan_filter_ptr_->Filter(edge_cloud, edge_cloud);
        cur_scan_filter_ptr_->Filter(surf_cloud, surf_cloud);

        // LOG(INFO) << "edge: " << edge_cloud->size() << ", surf: " << surf_cloud->size();
        if(has_init_eskf_)
        {
            match_ptr_->ScanMatch(edge_cloud, surf_cloud, T_w_i_eskf_ * TIL_, current_scan_, result_pos);
        }
        else
        {
            match_ptr_->ScanMatch(edge_cloud, surf_cloud, predict_pos, current_scan_, result_pos);
        }
    
        predict_pos = result_pos * last_pose.inverse() * result_pos;
        last_pose = result_pos;

        return result_pos;
    }

    bool Lio::GetLocalMap(CloudPtr& local_map)
    {
        if(b_use_loam_)
        {
            *local_map = *local_map_edge_;
            *local_map += *local_map_surf_;
            return true;    
        }

        if(local_map_ == nullptr)
            return false;

        *local_map = *local_map_;

        return true;
    }

    bool Lio::GetCurrentScan(CloudPtr& current_scan)
    {
        if(current_scan_ == nullptr)
            return false;

        *current_scan = *current_scan_;
        // pcl::transformPointCloud(*current_scan, *current_scan, TIL_.matrix().inverse().cast<float>());

        return true;
    }

    bool Lio::GetAllKeyFramePose(std::shared_ptr<std::deque<SE3>>& keyframe_poses)
    {
        if(estimated_poses_.empty()) 
            return false;
        
        *keyframe_poses = estimated_poses_;
        return true;
    }

    bool Lio::GetLatestKeyFramePose(SE3 &keyframe_pose)
    {
        if(estimated_poses_.empty())
            return false;
        
        keyframe_pose = last_kf_pose_;
        return true;
    }

    bool Lio::GetGlobalMap(CloudPtr& global_map)
    {
        std::vector<std::string> file_lists;
        CloudPtr temp_cloud(new PointCloudType);
        LOG(INFO) << "Reading PCD Files From " << lio_option_.key_frame_path_;
        TicToc get_file_name_time, read_pcd_time;
        get_file_name_time.tic();
        if (false == FileManager::GetFilesNameFromDirectory(file_lists, lio_option_.key_frame_path_))
        {
            return false;
        }
        LOG(INFO) << " GetFilesNameFromDirectory use time" << get_file_name_time.toc();

        read_pcd_time.tic();
        int index = 0;
        for (auto keyframe_file : file_lists)
        {
            // LOG(INFO) << keyframe_file;
            index = std::atoi(keyframe_file.substr(0, keyframe_file.length() - 4).c_str()); //get index
            // LOG(INFO) << "Index: " << index;
            ReadCloudFromPCDFile(lio_option_.key_frame_path_ + keyframe_file, temp_cloud);
            pcl::transformPointCloud(*temp_cloud, *temp_cloud, estimated_poses_[index -1].matrix());// transform from Lidar Axia to First Axia
            *global_map += *temp_cloud;
        }
        LOG(INFO) << " read pcd use time" << read_pcd_time.toc();

        local_map_filter_ptr_->Filter(global_map, global_map); // 暂时用local_map_filter_ptr_来滤波
        LOG(INFO) << "PointCloud Width/Size: " << global_map->width << "/" << global_map->size();
        temp_cloud = nullptr; // 手动释放内存（其实程序可自己释放）
        return true;
    }

    bool Lio::GetGlobalMap(CloudPtr& edge_map, CloudPtr& surf_map)
    {
        size_t kf_num = estimated_poses_.size();
        for(int idx {0}; idx < kf_num; ++idx)
        {
            CloudPtr edge_cloud(new PointCloudType), surf_cloud(new PointCloudType);
            std::string edge_cloud_path = lio_option_.loam_key_frame_path_ + "edge_" + std::to_string(idx + 1) + ".pcd";
            std::string surf_cloud_path = lio_option_.loam_key_frame_path_ + "surf_" + std::to_string(idx + 1) + ".pcd";

            if(FileManager::IsFileExist(edge_cloud_path))
            {
                ReadCloudFromPCDFile(edge_cloud_path, edge_cloud); 
                pcl::transformPointCloud(*edge_cloud, *edge_cloud, estimated_poses_.at(idx).matrix());
                *edge_map += *edge_cloud;
            }

            if(FileManager::IsFileExist(surf_cloud_path))
            {
                ReadCloudFromPCDFile(surf_cloud_path, surf_cloud);
                pcl::transformPointCloud(*surf_cloud, *surf_cloud, estimated_poses_.at(idx).matrix());

                *surf_map += *surf_cloud; 
            }
        }
        if(edge_map->points.empty() && surf_map->points.empty())
        {
            return false;
        }

        global_map_filter_ptr_->Filter(edge_map, edge_map);
        global_map_filter_ptr_->Filter(surf_map, surf_map);
        return true;
    }

    bool Lio::IsKeyframe(const SE3& current_pose) 
    {
        // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
        SE3 delta = last_kf_pose_.inverse() * current_pose;
        // LOG(INFO) << " delta.translation().norm() " << delta.translation().norm();
        return delta.translation().norm() > lio_option_.kf_distance_ ||
               delta.so3().log().norm() > lio_option_.kf_angle_deg_ * math::kDEG2RAD;
    }
    
}