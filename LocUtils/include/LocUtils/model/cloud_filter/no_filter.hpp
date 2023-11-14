//
// Created by Tian on 2023/10/01.
//

#pragma once

#include "LocUtils/model/cloud_filter/cloud_filter_interface.hpp"

namespace LocUtils 
{
  class NoFilter: public CloudFilterInterface 
  {
    public:
      NoFilter();

      bool Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) override;
  };
}
