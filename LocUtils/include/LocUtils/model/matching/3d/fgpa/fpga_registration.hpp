//
// Created by Tian on 2023/11/06.
//

#pragma once

#include <memory>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include "LocUtils/model/matching/3d/matching_interface.h"
#include "LocUtils/model/search_point/kdtree/kdtree.h"
#include "LocUtils/model/search_point/bfnn/bfnn.h"
namespace LocUtils
{

    class FpgaRegistration
    {
        public:
            FpgaRegistration() : 
                target_(new FullPointCloudType)
                ,source_(new FullPointCloudType)
            {

               
            
            }


            
        private:
            void SetSource(CloudPtr source);

            void PrintIcpMethodUse()
            {

            }
            
        private:
            
            Vec3d target_center_ {Vec3d::Zero()};
            Vec3d source_center_ {Vec3d::Zero()};

            FullCloudPtr target_{nullptr};
            FullCloudPtr source_{nullptr};
    };
}