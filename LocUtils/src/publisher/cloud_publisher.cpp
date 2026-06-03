/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "LocUtils/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace LocUtils {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudPtr&  cloud_ptr_input, double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudPtr&  cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::Publish(FullCloudPtr&  cloud_ptr_input, double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(FullCloudPtr&  cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization