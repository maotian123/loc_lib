#include "LocUtils/subscriber/scan_subscriber.hpp"

#include "glog/logging.h"

namespace LocUtils {
LaserScanSubscriber::LaserScanSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LaserScanSubscriber::msg_callback, this);
}

void LaserScanSubscriber::msg_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg_ptr) {
    buff_mutex_.lock();

    
    new_laser_scan_data_.push_back(*laser_scan_msg_ptr);

    buff_mutex_.unlock();
}

void LaserScanSubscriber::ParseData(std::deque<sensor_msgs::LaserScan>& deque_laser_scan_data) {
    DrainBuffer(buff_mutex_, new_laser_scan_data_, deque_laser_scan_data);
}
} // namespace data_input