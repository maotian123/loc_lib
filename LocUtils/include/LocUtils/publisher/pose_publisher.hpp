//
// Created by Tian on 2023/09/11.
//
#pragma once
#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/pose_data.hpp"
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace LocUtils
{
    class PosePublisher 
    {
        public:
            PosePublisher(ros::NodeHandle& nh, 
                            std::string topic_name, 
                            std::string base_frame_id,
                            int buff_size)
            :nh_(nh) {

                publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
                pos_.header.frame_id = base_frame_id;
            }
             
            PosePublisher() = default;

            template <typename S>
            void Publish(const Sophus::SE2<S> &se2)
            {
                Eigen::Matrix<S, 4, 4> mat = SE2TOMAT4(se2);
                PublishData(mat, velocity_data_, ros::Time::now());  
            }

            template <typename S>
            void Publish(const Sophus::SE3<S> &se3)
            {
                Eigen::Matrix<S, 4, 4> mat = SE3TOMAT4(se3);
                PublishData(mat, velocity_data_, ros::Time::now());  
            }

            bool HasSubscribers()
            {
                return publisher_.getNumSubscribers() != 0;
            }

        private:
            template <typename S>
            inline void PublishData(
                const Eigen::Matrix<S, 4, 4>& transform_matrix, 
                const VelocityData &velocity_data, 
                ros::Time time
            )
            {
                pos_.header.stamp = time;

                // set the pose
                pos_.pose.pose.position.x = transform_matrix(0,3);
                pos_.pose.pose.position.y = transform_matrix(1,3);
                pos_.pose.pose.position.z = transform_matrix(2,3);

                Quatd q;
                q = transform_matrix.template block<3,3>(0,0).template cast<double>();
                pos_.pose.pose.orientation.x = q.x();
                pos_.pose.pose.orientation.y = q.y();
                pos_.pose.pose.orientation.z = q.z();
                pos_.pose.pose.orientation.w = q.w();

                publisher_.publish(pos_);
            }

        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;

            VelocityData velocity_data_;
            geometry_msgs::PoseWithCovarianceStamped pos_;
    };
}
