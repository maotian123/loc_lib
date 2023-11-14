//
// Created by Tian on 2023/09/01.
//
#pragma once
#include "LocUtils/common/point_types.h"

namespace LocUtils
{
    class SearchPointInterface
    {
        public:
            virtual ~SearchPointInterface() = default;
            
            virtual bool SetTargetCloud(const CloudPtr &cloud) = 0;

            virtual std::vector<int> FindNearstPoints(const Vec3f& point, int k) = 0;

            virtual void FindCloud(const CloudPtr &cloud2, std::vector<std::pair<size_t, size_t>>& matches) = 0;

            virtual void SetEnableANN(bool use_ann = true, float alpha = 0.1)
            {
                
            }
    };
}




