//
// Created by Tian on 2023/09/11.
//
#pragma once

#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/pose_data.hpp"
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace LocUtils
{
    class OdometryPublisher 
    {
        public:
            OdometryPublisher(ros::NodeHandle& nh, 
                            std::string topic_name, 
                            std::string base_frame_id,
                            std::string child_frame_id,
                            int buff_size)
            :nh_(nh) {

                publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
                odometry_.header.frame_id = base_frame_id;
                odometry_.child_frame_id = child_frame_id;
            }
             
            OdometryPublisher() = default;

            template <typename S>
            void Publish(const Eigen::Matrix<S, 4, 4>& transform_matrix, double time)
            {
                ros::Time ros_time(time);
                PublishData(transform_matrix, velocity_data_, ros_time);
            }

            template <typename S>
            void Publish(const Eigen::Matrix<S, 4, 4>& transform_matrix)
            {
                PublishData(transform_matrix, velocity_data_, ros::Time::now());
            }

            template <typename S>
            void Publish(const Eigen::Matrix<S, 4, 4>& transform_matrix, const VelocityData &velocity_data, double time)
            {
                ros::Time ros_time(time);
                PublishData(transform_matrix, velocity_data, ros_time);
            }

            template <typename S>
            void Publish(const Eigen::Matrix<S, 4, 4>& transform_matrix, const VelocityData &velocity_data)
            {
                PublishData(transform_matrix, velocity_data, ros::Time::now());
            }

            template <typename S>
            void Publish(const Eigen::Matrix<S, 4, 4>& transform_matrix, const Eigen::Vector3f& vel, double time)
            {
                ros::Time ros_time(time);

                velocity_data_.linear_velocity.x = vel.x();
                velocity_data_.linear_velocity.y = vel.y();
                velocity_data_.linear_velocity.z = vel.z();
                
                PublishData(transform_matrix, velocity_data_, ros_time);
            }
            
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
            

            template <typename S>
            void Publish(const Sophus::SE3<S> &se3, double time)
            {
                ros::Time ros_time(time);
                Eigen::Matrix<S, 4, 4> mat = SE3TOMAT4(se3);
                PublishData(mat, velocity_data_, ros_time);  
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
                odometry_.header.stamp = time;

                // set the pose
                odometry_.pose.pose.position.x = transform_matrix(0,3);
                odometry_.pose.pose.position.y = transform_matrix(1,3);
                odometry_.pose.pose.position.z = transform_matrix(2,3);

                Quatd q;
                q = transform_matrix.template block<3,3>(0,0).template cast<double>();
                odometry_.pose.pose.orientation.x = q.x();
                odometry_.pose.pose.orientation.y = q.y();
                odometry_.pose.pose.orientation.z = q.z();
                odometry_.pose.pose.orientation.w = q.w();

                // set the twist:
                // a. linear:
                odometry_.twist.twist.linear.x = velocity_data.linear_velocity.x;
                odometry_.twist.twist.linear.y = velocity_data.linear_velocity.y;
                odometry_.twist.twist.linear.z = velocity_data.linear_velocity.z;
                // b. angular:
                odometry_.twist.twist.angular.x = velocity_data.angular_velocity.x;
                odometry_.twist.twist.angular.y = velocity_data.angular_velocity.y;
                odometry_.twist.twist.angular.z = velocity_data.angular_velocity.z;

                publisher_.publish(odometry_);

            }

        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;

            VelocityData velocity_data_;
            nav_msgs::Odometry odometry_;
    };
}
