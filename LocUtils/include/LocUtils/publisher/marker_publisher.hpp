//
// Created by Tian on 2023/09/07.
//

#pragma once
#include "LocUtils/common/eigen_types.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace LocUtils
{
    class MarkerPublisher 
    {
        public:
            MarkerPublisher(
                ros::NodeHandle& nh,
                std::string topic_name,
                std::string frame_id,
                size_t buff_size
            );
            MarkerPublisher() = default;

            void Publish(const std::vector<Vec2f> &marker);
            void Publish(const std::vector<Vec2f> &marker, const Vec3f &rgb);
            void Publish(const std::vector<Vec3f> &marker);
            void Publish(const std::vector<Vec3f> &marker, const Vec3f &rgb);

        private:

            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id_;
            std::string topic_name_;
            visualization_msgs::MarkerArray maker_;
    };
}