/*
 * @Date: 2023-10-26 21:56:16
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-27 18:05:53
 * @FilePath: /LocUtils/src/subscriber/gnss_subscriber.cpp
 */
#include "LocUtils/subscriber/gnss_subscriber.hpp"
#include "glog/logging.h"

namespace LocUtils{
GnssSubscriber::GnssSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) 
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::msg_callback, this);
}

void GnssSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr) {
    buff_mutex_.lock();

    // convert ROS IMU to GeographicLib compatible GNSS message:
    GnssData gnss_data;
    gnss_data.timestamp_ = gnss_msg_ptr->header.stamp.toSec();
    gnss_data.latitude_ = gnss_msg_ptr->latitude;
    gnss_data.longitude_ = gnss_msg_ptr->longitude;
    gnss_data.altitude_ = gnss_msg_ptr->altitude;
    // add new message to buffer:
    new_gnss_data_.push_back(gnss_data);
    
    buff_mutex_.unlock();
}

void GnssSubscriber::ParseData(std::deque<GnssData>& deque_gnss_data) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_gnss_data_.size() > 0) {
        deque_gnss_data.insert(deque_gnss_data.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }

    buff_mutex_.unlock();
}
}