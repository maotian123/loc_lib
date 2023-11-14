/*
 * @Description: 在ros中发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LOCUTILS_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LOCUTILS_PUBLISHER_CLOUD_PUBLISHER_HPP_
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "LocUtils/sensor_data/cloud_data.hpp"

namespace LocUtils {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    CloudPublisher() = default;

    void Publish(CloudPtr& cloud_ptr_input, double time);
    void Publish(CloudPtr& cloud_ptr_input);

    void Publish(FullCloudPtr& cloud_ptr_input, double time);
    void Publish(FullCloudPtr& cloud_ptr_input);

    bool HasSubscribers();
  
  private:
    void PublishData(CloudPtr& cloud_ptr_input, ros::Time time);
    void PublishData(FullCloudPtr& cloud_ptr_input, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif