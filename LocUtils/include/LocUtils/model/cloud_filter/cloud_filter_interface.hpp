//
// Created by Tian on 2023/10/01.
//
#pragma once

#include "LocUtils/common/point_cloud_utils.h"
namespace LocUtils 
{
  class CloudFilterInterface 
  {
    public:
      virtual ~CloudFilterInterface() = default;

      virtual bool Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) = 0;
  };
}
