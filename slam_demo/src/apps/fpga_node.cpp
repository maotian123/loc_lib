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

std::deque<LocUtils::CloudDataFullType> last_lidar_line_buff;
std::deque<LocUtils::CloudDataFullType> curr_lidar_line_buff (16);

LocUtils::FullPointType FindNearstP(int p_idx, int ring_idx)
{
    Vec3d cur_p = LocUtils::ToVec3d(curr_lidar_line_buff[ring_idx].cloud_ptr->points[p_idx]);
    double dis{10000.0};
    LocUtils::FullPointType result_point;
    Vec3d result_p;
    result_p.x() = 1000.0;
    result_p.y() = 1000.0;
    result_p.z() = 1000.0;
    for (int ring_interval{-2}; ring_interval < 2; ++ring_interval)
    {
        int select_ring = ring_interval + ring_idx;

        if(select_ring < 0 || select_ring > 15)
            continue;

        for(int p_interval{-5}; p_interval < 5; ++p_interval)
        {
            int select_p = p_interval + p_idx;
            const int point_size = last_lidar_line_buff[select_ring].cloud_ptr->points.size() - 1;

            if (select_p < 0 || select_p > point_size)
                continue;

            Vec3d last_p = last_lidar_line_buff[select_ring].cloud_ptr->points[select_p].getArray3fMap().cast<double>();
            double temp_dis = std::sqrt((last_p - cur_p).squaredNorm());
            if(temp_dis < dis)
            {
                dis = temp_dis;
                result_p = last_p;
                result_point = last_lidar_line_buff[select_ring].cloud_ptr->points[select_p];
            }
        }
    }

    return result_point;
}

void TestLidarStruct(int argc, char *argv[])
{
    ros::init(argc, argv, "fpga_node");
    ros::NodeHandle nh;

    LocUtils::LidarType lidar_type {LocUtils::LidarType::ROBOSENSE};

    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr =
        std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000, lidar_type);

    std::shared_ptr<LocUtils::CloudPublisher> line1_pub_ptr_
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/line1", "/map", 10000);
 
    std::deque<LocUtils::CloudDataFullType> cloud_full_data_buff;
    LocUtils::CloudDataFullType cur_lidar;
    SE3 lidar_pos{SE3()};
    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        std::deque<LocUtils::CloudDataFullType> cur_lidar_line_buff(16);
        LocUtils::CloudDataFullType pub_lidar;
        cloud_sub_ptr->ParseData(cloud_full_data_buff);

        if(cloud_full_data_buff.empty())
            continue;
        
        cur_lidar = cloud_full_data_buff.front();
        for(auto & p:cur_lidar.cloud_ptr->points)
        {
            int ring = p.ring;
            cur_lidar_line_buff[ring].cloud_ptr->points.emplace_back(p);
        }

        for (int i{0}; i < 100; ++i)
        {
            pub_lidar.cloud_ptr->points.emplace_back(cur_lidar_line_buff[0].cloud_ptr->points.at(i));
        }
        line1_pub_ptr_->Publish(pub_lidar.cloud_ptr);
        cloud_full_data_buff.pop_front();
    }
}

int TestRingSearch(int argc, char *argv[])
{
    ros::init(argc, argv, "fpga_node");
    ros::NodeHandle nh;

    LocUtils::LidarType lidar_type {LocUtils::LidarType::ROBOSENSE};

    std::shared_ptr<LocUtils::CloudSubscriber> cloud_sub_ptr =
        std::make_shared<LocUtils::CloudSubscriber>(nh, "/rslidar_points", 10000, lidar_type);

    std::shared_ptr<LocUtils::CloudPublisher> pub_source_cloud_ptr
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/source", "/map", 10000);
    
    std::shared_ptr<LocUtils::CloudPublisher> pub_target_cloud_ptr
                    = std::make_shared<LocUtils::CloudPublisher>(nh, "/target", "/map", 10000);
 
    std::deque<LocUtils::CloudDataFullType> cloud_full_data_buff;
    LocUtils::CloudDataFullType cur_lidar;
    SE3 lidar_pos{SE3()};
    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        curr_lidar_line_buff.clear();
        curr_lidar_line_buff.resize(16);
        LocUtils::CloudDataFullType pub_source_lidar;
        LocUtils::CloudDataFullType pub_target_lidar;
        cloud_sub_ptr->ParseData(cloud_full_data_buff);

        if(cloud_full_data_buff.empty())
            continue;
        
        cur_lidar = cloud_full_data_buff.front();
        for(auto & p:cur_lidar.cloud_ptr->points)
        {
            // lidar_line_buff[p.ring].cloud_ptr->points.emplace_back(p);
            
            int ring = p.ring;
            curr_lidar_line_buff[ring].cloud_ptr->points.emplace_back(p);
        }
        static bool init_fisrt_scan{false};

        if(!init_fisrt_scan)
        {
            last_lidar_line_buff = curr_lidar_line_buff;
            init_fisrt_scan = true;
        }

        Mat6d H {Mat6d::Zero()};
        Vec6d B {Vec6d::Zero()};
        SE3 predic_pose{SE3()};


        for (int i{0}; i < 16; ++i)
        {
            for (int j{0}; j < curr_lidar_line_buff[i].cloud_ptr->points.size(); ++j)
            {
                Vec3d p = LocUtils::ToVec3d(curr_lidar_line_buff[i].cloud_ptr->points[j]);

                LocUtils::FullPointType q_point = FindNearstP(j, i);
                Vec3d q = LocUtils::ToVec3d(q_point);
                double dis = std::sqrt((p - q).squaredNorm());

                if(dis > 1.0)
                    continue;

                pub_source_lidar.cloud_ptr->points.emplace_back(curr_lidar_line_buff[i].cloud_ptr->points[j]);
                pub_target_lidar.cloud_ptr->points.emplace_back(q_point);
                
                // Eigen::Matrix<double, 3, 6> J;
                // auto q = LocUtils::ToVec3d(cur_lidar_line_buff[i].cloud_ptr->points[j]);
                // Vec3d e = last_p - cur_p;
                // J.block<3, 3>(0, 0) = predic_pose.so3().matrix() * SO3::hat(q);
                // J.block<3, 3>(0, 3) = -Mat3d::Identity();

                // H += J.transpose() * J;
                // B += -J.transpose() * e;
                // break;    
                    
            }
        }
        

        // if (H.determinant() == 0)
        // {
        //     LOG(INFO) << "H not ok";
        // }
        // else
        // {
        //     Vec6d dx = H.inverse() * B;
        //     predic_pose.so3() = predic_pose.so3() * SO3::exp(dx.head<3>());
        //     predic_pose.translation() += dx.tail<3>();

        //     lidar_pos = predic_pose * lidar_pos;
        //     LOG(INFO) << lidar_pos.matrix();
        //     pcl::transformPointCloud(*cur_lidar.cloud_ptr, *cur_lidar.cloud_ptr, lidar_pos.matrix());
        // }

        cloud_full_data_buff.pop_front();
        pub_source_cloud_ptr->Publish(pub_source_lidar.cloud_ptr);
        pub_target_cloud_ptr->Publish(pub_target_lidar.cloud_ptr);
        last_lidar_line_buff = curr_lidar_line_buff;

        r.sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    TestRingSearch(argc, argv);
}