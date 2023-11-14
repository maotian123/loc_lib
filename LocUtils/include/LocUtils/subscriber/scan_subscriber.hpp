/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-06-30 17:34:24
 * @LastEditors: lxy
 * @LastEditTime: 2023-06-30 17:35:28
 */
#ifndef LOCUTILS_SUBSCRIBER_SCAN_SUBSCRIBER_HPP_
#define LOCUTILS_SUBSCRIBER_SCAN_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


namespace LocUtils {
class LaserScanSubscriber {
  public:
    LaserScanSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LaserScanSubscriber() = default;
    void ParseData(std::deque<sensor_msgs::LaserScan>& deque_laser_scan_data);

  private:
    void msg_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<sensor_msgs::LaserScan> new_laser_scan_data_;

    std::mutex buff_mutex_;
};
}

#endif