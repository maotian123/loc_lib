//
// Created by Tian on 2023/10/28.
//

#include "LocUtils/slam/3d/loc.hpp"

namespace LocUtils
{
    Loc::Loc()
        :global_map_ptr_(new PointCloudType)
        ,current_scan_ptr_(new PointCloudType)
        ,local_map_ptr_(new PointCloudType)
    {
        match_ptr_ = std::make_shared<NdtRegistration>();
        InitCloudFilter();
        InitGlobalMap();
        InitEskf();
        ResetLocalMap(0.0, 0.0, 0.0);
    }
    
    Loc::Loc(LocOptions loc_option)
            :loc_option_(loc_option)
            ,global_map_ptr_(new PointCloudType)
            ,current_scan_ptr_(new PointCloudType)
            ,local_map_ptr_(new PointCloudType)
    {
        match_ptr_ = std::make_shared<NdtRegistration>();
        InitCloudFilter();
        InitGlobalMap();
        InitEskf();
        ResetLocalMap(0.0, 0.0, 0.0);
    }

    Loc::Loc(LocOptions loc_option, NdtOptions ndt_option)
            :loc_option_(loc_option)
            , ndt_option_(ndt_option)
            ,global_map_ptr_(new PointCloudType)
            ,current_scan_ptr_(new PointCloudType)
            ,local_map_ptr_(new PointCloudType)
    {
        match_ptr_ = std::make_shared<NdtRegistration>(ndt_option_);
        InitCloudFilter();
        InitGlobalMap();
        InitEskf();
        ResetLocalMap(0.0, 0.0, 0.0);
    }

    Loc::Loc(LocOptions loc_option, IcpOptions icp_option)
            :loc_option_(loc_option)
            ,icp_option_(icp_option)
            ,global_map_ptr_(new PointCloudType)
            ,current_scan_ptr_(new PointCloudType)
            ,local_map_ptr_(new PointCloudType)
    {
        match_ptr_ = std::make_shared<IcpRegistration>(icp_option_);
        InitCloudFilter();
        InitGlobalMap();
        InitEskf();
        ResetLocalMap(0.0, 0.0, 0.0);

    }

    Loc::Loc(LocOptions loc_option, LoamOption loam_option)
            :loc_option_(loc_option)
            ,loam_option_(loam_option)
            ,global_map_ptr_(new PointCloudType)
            ,current_scan_ptr_(new PointCloudType)
            ,local_map_ptr_(new PointCloudType)
    {
        b_use_loam_ = true;
        match_ptr_ = std::make_shared<LoamRegistration>(loam_option_);
        InitCloudFilter();
        InitGlobalMap();
        InitEskf();
        ResetLocalMap(0.0, 0.0, 0.0);

    }

    bool Loc::InitImu(std::deque<IMU> &imu_buff)
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

    void Loc::InitCloudFilter()
    {
        global_map_filter_ptr_ = std::make_shared<VoxelFilter>(loc_option_.global_map_filter_);
        cur_scan_filter_ptr_ = std::make_shared<VoxelFilter>(loc_option_.cur_scan_filter_);

        box_filter_ptr_ = std::make_shared<BoxFilter>(loc_option_.box_filter_size_[0], 
                                                      loc_option_.box_filter_size_[1],
                                                      loc_option_.box_filter_size_[2]);
    }

    void Loc::InitEskf()
    {
        static_imu_init_ptr_ = std::make_shared<LocUtils::StaticIMUInit>();
        eskf_ptr_ = std::make_shared<LocUtils::ESKFD>();

        Vec3d T_imu_lidar = math::VecFromArray(loc_option_.ext_t);
        Mat3d R_imu_lidar = math::RpyToRotM2(loc_option_.ext_r[0], loc_option_.ext_r[1], loc_option_.ext_r[2]);

        TIL_ = SE3(R_imu_lidar, T_imu_lidar);
        LOG(INFO) << "---------- TIL_ ---------" << TIL_.matrix();

    }

    bool Loc::HasNewGlobalMap()
    {
        return b_has_new_global_map_;
    }

    bool Loc::HasNewLocalMap()
    {
        return b_has_new_local_map_;
    }

    bool Loc::HasInit()
    {
        return b_has_init_;
    }

    bool Loc::GetCurrentScan(CloudPtr& current_scan)
    {
        if(current_scan_ptr_ == nullptr)
            return false;

        *current_scan = *current_scan_ptr_;
        // pcl::transformPointCloud(*current_scan, *current_scan, TIL_.matrix().inverse().cast<float>());

        return true;   
    }

    CloudPtr& Loc::GetGlobalMap()
    {
        b_has_new_global_map_ = false;
        return global_map_ptr_;
    }

    CloudPtr& Loc::GetLocalMap()
    {
        b_has_new_local_map_ = false;
        return local_map_ptr_;
    }

    void Loc::SetInitPose(const SE3 &pose)
    {
        cur_pos_ = pose;
        init_pos_ = pose;
        b_has_init_ = true;
        if(has_init_eskf_)
        {
            T_w_i_eskf_ = pose * TIL_.inverse();
            Vec3d g_w_i = T_w_i_eskf_ * eskf_ptr_->GetGravity();
            LOG(INFO) << " g_w_i : " << g_w_i;
            eskf_ptr_->SetX(T_w_i_eskf_, g_w_i);
        }
    
        ResetLocalMap(pose.translation().x() , pose.translation().y(), pose.translation().z());
    }

    bool Loc::ResetLocalMap(float x, float y, float z)
    {
        std::vector<float> origin = {x, y, z};

        box_filter_ptr_->SetOrigin(origin);
        box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

        match_ptr_->SetInputTarget(local_map_ptr_);

        b_has_new_local_map_ = true;
        LOG(INFO) << " local_map_ptr_ " << local_map_ptr_->points.size();
        std::vector<float> edge = box_filter_ptr_->GetEdge();
        std::cout << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;
        return true;
    }

    void Loc::Update(const CloudData &input_cloud)
    {
        static SE3 last_pose = init_pos_;
        static SE3 predict_pos = init_pos_;
        cur_cloud_time_ = input_cloud.time;
     
        CloudPtr cloud_no_nan_ptr(new PointCloudType);
        current_scan_ptr_.reset(new PointCloudType);
        SE3 result_pos;
        cloud_no_nan_ptr = RemoveNanPoint(input_cloud.cloud_ptr);
        cur_scan_filter_ptr_->Filter(cloud_no_nan_ptr, cloud_no_nan_ptr);
        if(has_init_eskf_)
        {
            match_ptr_->ScanMatch(cloud_no_nan_ptr, T_w_i_eskf_ * TIL_, current_scan_ptr_, result_pos);
            // LOG(INFO) << " result_pos " << result_pos.matrix();
            T_w_i_eskf_ = result_pos * TIL_.inverse();
            eskf_ptr_->ObserveSE3(T_w_i_eskf_);
            result_pos = eskf_ptr_->GetNominalSE3() * TIL_;
        }
        else
        {
            match_ptr_->ScanMatch(cloud_no_nan_ptr, predict_pos, current_scan_ptr_, result_pos);
        }

        predict_pos = result_pos * last_pose.inverse() * result_pos;
        last_pose = result_pos;

        std::vector<float> edge = box_filter_ptr_->GetEdge();
        for (int i = 0; i < 3; i++) {
            if (
                fabs(result_pos.translation()[i] - edge.at(2 * i)) > 50.0 &&
                fabs(result_pos.translation()[i] - edge.at(2 * i + 1)) > 50.0
            ) {
                continue;
            }
                
            ResetLocalMap(result_pos.translation()[0], result_pos.translation()[1], result_pos.translation()[2]);
            break;
        }
    }

    void Loc::Update(const CloudDataFullType &input_cloud)
    {
        static SE3 last_pose = SE3();
        static SE3 predict_pos = SE3();
        
    }

    void Loc::Update(const IMU &imu_data)
    {
        double dt = imu_data.timestamp_ - cur_cloud_time_;
        if(imu_data.timestamp_ < cur_cloud_time_)
        {
            // LOG(INFO) << "do not use this imu data dt = " << dt;
            return;
        }
        eskf_ptr_->Predict(imu_data);
        T_w_i_eskf_ = eskf_ptr_->GetNominalSE3();
    }

    bool Loc::InitGlobalMap()
    {
        if(!boost::filesystem::exists(loc_option_.map_path_))
        {
            LOG(INFO) << loc_option_.map_path_ << " does not exits ";
            return false;
        }

        pcl::io::loadPCDFile(loc_option_.map_path_, *global_map_ptr_);

        global_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
        LOG(INFO) << "\tLoad Global Map, size:" << global_map_ptr_->points.size();

        b_has_new_global_map_ = true;
        return true;
    }

    SE3 Loc::GetCurrPos()
    {
        SE3 pose = T_w_i_eskf_ * TIL_;
        return pose;
    }

    void Loc::AlignWithBoxMap(const CloudPtr &source_cloud)
    {
        
    }
}