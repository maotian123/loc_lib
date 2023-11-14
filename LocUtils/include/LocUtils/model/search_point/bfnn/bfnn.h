//
// Created by Tian on 2023/08/31.
//
#pragma once
#include "LocUtils/model/search_point/search_point_interface.h"

namespace LocUtils
{
    class BfnnRegistration : public SearchPointInterface
    {
        public:
            BfnnRegistration(bool use_multi = false);

            bool SetTargetCloud(const CloudPtr &cloud) override;

            std::vector<int> FindNearstPoints(const Vec3f& point, int k) override;

            void FindCloud(const CloudPtr &cloud2, std::vector<std::pair<size_t, size_t>>& matches) override;

        private:
            int BfnnPoint(const Vec3f& point);

        private:
            struct IndexAndDis 
            {
                IndexAndDis() {}
                IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
                int index_ = 0;
                double dis2_ = 0;
            };
            bool b_use_m_{false};

            CloudPtr source_cloud_;
    };
}