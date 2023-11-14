//
// Created by Tian on 2023/09/05.
//

#ifndef LOCUTILS_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LOCUTILS_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <deque>
#include <mutex>
#include <thread>
#include <type_traits>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "LocUtils/sensor_data/cloud_data.hpp"
#include "LocUtils/common/point_cloud_utils.h"
namespace LocUtils {

enum class LidarType
{
    ROBOSENSE,
    VELODYNE,
    LIVOX,
    NOTYPE
}; 

//点云转换
class CloudConver
{
  public:
    CloudConver();

    bool Conver(const CloudData &input, CloudData &output);
    bool Conver(const CloudDataRsLidar &input, CloudDataFullType &output);
    bool Conver(const CloudDataVelodyneLidar &input, CloudDataFullType &output);
  
};

CloudConver::CloudConver()
{

}

//不同雷达的sub
class CloudSubscriber {
  public:

    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, LidarType lidar_type);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);
    void ParseData(std::deque<CloudDataFullType>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
    bool AddNormal(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
    bool AddRslidar(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;
    std::deque<CloudDataFullType> new_cloud_full_data_;
    LidarType lidar_type_ {LidarType::NOTYPE};
    CloudConver cloud_conver_;
    std::string topic_name_;
    std::mutex buff_mutex_;
};

}

#endif