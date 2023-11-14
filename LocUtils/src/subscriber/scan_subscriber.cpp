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
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_laser_scan_data_.size() > 0) {
        deque_laser_scan_data.insert(deque_laser_scan_data.end(), new_laser_scan_data_.begin(), new_laser_scan_data_.end());
        new_laser_scan_data_.clear();
    }
    
    buff_mutex_.unlock();
}
} // namespace data_input