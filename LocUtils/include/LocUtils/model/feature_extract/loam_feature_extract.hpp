//
// Created by Tian on 2023/10/10.
//

#pragma once

#include "LocUtils/common/point_types.h"
#include "LocUtils/common/point_cloud_utils.h"
#include "LocUtils/common/eigen_types.h"
#include "LocUtils/sensor_data/cloud_data.hpp"
namespace LocUtils
{
    struct IdAndValue {
        IdAndValue() {}
        IdAndValue(int id, double value) : id_(id), value_(value) {}
        int id_ = 0;
        double value_ = 0;  // 曲率
    };

    struct LoamFeatureOptions
    {
        size_t num_scan_ {16};
        /* data */
    };

    class LoamFeatureExtract
    {
        public:
            LoamFeatureExtract(LoamFeatureOptions option);

            //提取        
            void Extract(FullCloudPtr &pc_in, CloudPtr &pc_out_edge, CloudPtr &pc_out_surf);

            /**
             * 对单独一段区域提取角点和面点
             * @param pc_in
             * @param cloud_curvature
             * @param pc_out_edge
             * @param pc_out_surf
             */
            void ExtractFromSector(const CloudPtr& pc_in, std::vector<IdAndValue>& cloud_curvature, CloudPtr& pc_out_edge,
                                CloudPtr& pc_out_surf);
        private:
            LoamFeatureOptions option_;
    };
}