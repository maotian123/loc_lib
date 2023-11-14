//
// Created by Tian on 2023/09/06.
//
#pragma once

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "LocUtils/common/eigen_types.h"

namespace LocUtils
{
    class TFListener
    {

        public:
            TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
            TFListener() = default;

            bool Lookup3DData(Mat4f& transform_matrix);
            bool Lookup2DData(Vec3f& xyYaw);
            
        private:
            bool TransformToMatrix(const tf::StampedTransform& transform, Mat4f& transform_matrix);
        
        private:
            ros::NodeHandle nh_;
            tf::TransformListener listener_;
            std::string base_frame_id_;
            std::string child_frame_id_;
    };
    
}