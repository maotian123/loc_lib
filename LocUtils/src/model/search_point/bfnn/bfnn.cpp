#include "LocUtils/model/search_point/bfnn/bfnn.h"

#include <glog/logging.h>
namespace LocUtils
{
    BfnnRegistration::BfnnRegistration(bool use_multi):
        b_use_m_(use_multi)
        ,source_cloud_(new PointCloudType)
    {
        LOG(INFO) << "CREATE BFNN SEARCH ";
    }

    bool BfnnRegistration::SetTargetCloud(const CloudPtr &cloud)
    {
        if(cloud->points.empty())
        {
            return false;
        }
        
        pcl::copyPoint(*cloud, *source_cloud_);
        return true;
    }

    std::vector<int> BfnnRegistration::FindNearstPoints(const Vec3f& point, int k)
    {
        std::vector<IndexAndDis> index_and_dis(source_cloud_->size());
        const int cloud_size = source_cloud_->points.size();
        // LOG(INFO) << " source point size " << cloud_size;
        
        for (int i {0}; i < cloud_size; ++i) 
        {
            index_and_dis[i] = {i, (source_cloud_->points[i].getVector3fMap() - point).squaredNorm()};
        }

        std::sort(index_and_dis.begin(), index_and_dis.end(),
              [](const auto& d1, const auto& d2) { return d1.dis2_ < d2.dis2_; });
    
        std::vector<int> ret;

        std::transform(index_and_dis.begin(), index_and_dis.begin() + k, std::back_inserter(ret),
                   [](const auto& d1) { return d1.index_; });

        if(!ret.empty())
        {
           
            // LOG(INFO) << "nearst dis : " << index_and_dis.front().dis2_;
        }

        return ret;
    }

    void BfnnRegistration::FindCloud(const CloudPtr &cloud2, 
                                     std::vector<std::pair<size_t, size_t>>& matches)
    {

    }

    int BfnnRegistration::BfnnPoint(const Vec3f& point)
    {
        return std::min_element(source_cloud_->points.begin(), source_cloud_->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           source_cloud_->points.begin();
    }

}




