#include "LocUtils/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace LocUtils {

bool CloudConver::Conver(const CloudData &input, CloudData &output)
{
  output.time = input.time;
  for (size_t i{0}; i < input.cloud_ptr->points.size(); ++i)
  {
    auto &src = input.cloud_ptr->points.at(i);
    if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
      continue;
    if(PointDistance(src) < 4.0)
    {
      continue; 
    }  
    PointType p = src;
    p.x = src.x;
    p.y = src.y;
    p.z = src.z;
    p.intensity = src.intensity;
    output.cloud_ptr->points.emplace_back(p);
  }
  // LOG(INFO) << input.cloud_ptr->points.size();
  // LOG(INFO) << output.cloud_ptr->points.size();
  return true;
}

bool CloudConver::Conver(const CloudDataRsLidar &input, CloudDataFullType &output)
{
  //速腾雷达ros时间戳是最后一个雷达点的时间戳
  // output.cloud_ptr->points.resize(input.cloud_ptr->points.size());
  double timespan = input.cloud_ptr->points.back().timestamp - input.cloud_ptr->points.front().timestamp;
  for (size_t i {0}; i < input.cloud_ptr->points.size(); ++i)
  {
    auto &src = input.cloud_ptr->points.at(i);
    if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
      continue;

    if(PointDistance(src) < 4.0)
    {
      continue;
    }
    FullPointType p;
    p.x = src.x;
    p.y = src.y;
    p.z = src.z;
    p.intensity = src.intensity;
    p.ring = (uint8_t)src.ring;
    p.time_span = timespan;
    p.time_intervel = (src.timestamp - input.cloud_ptr->points.front().timestamp)/timespan;
    output.cloud_ptr->points.emplace_back(p);
  }
  output.time = input.time;
  return true;  
}

bool CloudConver::Conver(const CloudDataVelodyneLidar &input, CloudDataFullType &output)
{

}


CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh)
    ,topic_name_(topic_name)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, 
                                size_t buff_size, LidarType lidar_type)
                                :nh_(nh)
                                ,topic_name_(topic_name)
                                ,lidar_type_(lidar_type)
{
    assert(lidar_type != LidarType::NOTYPE);
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
  buff_mutex_.lock();

  // 将不同雷达pcl类型转换为 deque所需要的 自定义类型
  // 为了适配 loam 所需的 fullcloudtype 
  // 直接法 pointcloudtype
  switch (lidar_type_)
  {
    case LidarType::ROBOSENSE:
        AddRslidar(cloud_msg_ptr);
        break;
    case LidarType::NOTYPE:
        AddNormal(cloud_msg_ptr);
    default:
        break;
  }
  buff_mutex_.unlock();
}

bool CloudSubscriber::AddNormal(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
    CloudData normal_cloud_data;
    CloudData new_cloud_data;
    normal_cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(normal_cloud_data.cloud_ptr));
    if(cloud_conver_.Conver(normal_cloud_data, new_cloud_data))
    {
        new_cloud_data_.push_back(new_cloud_data);

        return true;
    }  
}

bool CloudSubscriber::AddRslidar(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
    // cloud_conver_.Conver()
    // ros消息转换为不同雷达的pcl类型
    CloudDataRsLidar rslidar_cloud_data;
    CloudDataFullType new_cloud_data;
    rslidar_cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(rslidar_cloud_data.cloud_ptr));

    if(cloud_conver_.Conver(rslidar_cloud_data, new_cloud_data))
    {
        new_cloud_full_data_.push_back(new_cloud_data);

        return true;
    }  
    return true;
}

void CloudSubscriber::ParseData(std::deque<CloudData>& deque_cloud_data) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_cloud_data_.size() > 0) {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudDataFullType>& deque_cloud_data)
{
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_cloud_full_data_.size() > 0) {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_full_data_.begin(), new_cloud_full_data_.end());
        new_cloud_full_data_.clear();
    }
    
    buff_mutex_.unlock();
}

} // namespace data_input