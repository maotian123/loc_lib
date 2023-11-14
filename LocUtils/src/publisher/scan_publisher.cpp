#include "LocUtils/publisher/scan_publisher.hpp"
#include "glog/logging.h"


namespace LocUtils {
LaserScanPubscriber::LaserScanPubscriber(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::LaserScan>(topic_name, buff_size);
}

void LaserScanPubscriber::Publish(sensor_msgs::LaserScan&  cloud_ptr_input, double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void LaserScanPubscriber::Publish(sensor_msgs::LaserScan&  cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void LaserScanPubscriber::PublishData(sensor_msgs::LaserScan&  cloud_ptr_input, ros::Time time) {
    sensor_msgs::LaserScan cur_laser_scan_;
    cur_laser_scan_.header.stamp = time;
    cur_laser_scan_.header.frame_id = frame_id_;
    cur_laser_scan_.angle_max = cloud_ptr_input.angle_max;
    cur_laser_scan_.angle_min = cloud_ptr_input.angle_min;
    cur_laser_scan_.angle_increment = cloud_ptr_input.angle_increment;
    cur_laser_scan_.intensities = cloud_ptr_input.intensities;
    cur_laser_scan_.range_max = cloud_ptr_input.range_max;
    cur_laser_scan_.range_min = cloud_ptr_input.range_min;
    cur_laser_scan_.ranges = cloud_ptr_input.ranges;
    cur_laser_scan_.scan_time = cloud_ptr_input.scan_time;
    cur_laser_scan_.time_increment = cloud_ptr_input.time_increment;

    publisher_.publish(cur_laser_scan_);
}

bool LaserScanPubscriber::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization






















