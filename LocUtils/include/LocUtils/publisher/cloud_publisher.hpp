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
    template <typename CloudPtrT>
    void PublishData(CloudPtrT& cloud_ptr_input, ros::Time time)
    {
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif