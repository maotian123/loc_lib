/*
 * @Date: 2023-10-26 21:52:28
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-27 18:04:30
 * @FilePath: /LocUtils/include/LocUtils/subscriber/gnss_subscriber.hpp
 */

#ifndef LOCUTILS_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LOCUTILS_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "LocUtils/sensor_data/gnss_data.hpp"

namespace LocUtils {
class GnssSubscriber {
  public:
    GnssSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GnssSubscriber() = default;
    void ParseData(std::deque<GnssData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GnssData> new_gnss_data_;

    std::mutex buff_mutex_; 
};
}
#endif