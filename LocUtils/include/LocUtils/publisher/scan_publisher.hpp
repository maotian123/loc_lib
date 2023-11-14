/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-06-30 17:41:59
 * @LastEditors: lxy
 * @LastEditTime: 2023-06-30 17:48:37
 */
#ifndef LOCUTILS_PUBLISHER_SCAN_PUBLISHER_HPP_
#define LOCUTILS_PUBLISHER_SCAN_PUBLISHER_HPP_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace LocUtils {
class LaserScanPubscriber {
  public:
    LaserScanPubscriber(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    LaserScanPubscriber() = default;

    void Publish(sensor_msgs::LaserScan& laser_input, double time);
    void Publish(sensor_msgs::LaserScan& laser_input);

    bool HasSubscribers();
  
  private:
    void PublishData(sensor_msgs::LaserScan& laser_input, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif