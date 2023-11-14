#pragma once

#include <deque>
#include <mutex>
#include <thread>
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "LocUtils/common/pose_data.hpp"

namespace LocUtils
{
    class OdometrySubscriber 
    {
        public:
            OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
            OdometrySubscriber() = default;
            void ParseData(std::deque<PoseData>& deque_pose_data);

        private:
            void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
            
        private:
            ros::NodeHandle nh_;
            ros::Subscriber subscriber_;
            std::deque<PoseData> new_pose_data_;

            std::mutex buff_mutex_; 
        };
}