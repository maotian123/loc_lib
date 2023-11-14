//
// Created by Tian on 2023/10/31.
//
#include "LocUtils/model/cloud_filter/cloud_filter_interface.hpp"
#include "LocUtils/common/point_types.h"
#include <pcl/filters/crop_box.h>

namespace LocUtils
{
    class BoxFilter: public CloudFilterInterface
    {
        public:
            BoxFilter(float step_x = 150.f, float step_y = 150.f, float step_z = 150.f);
            BoxFilter() = default;
            /*** 
             * @description: box filter 点云接口
             * @param {CLOUD_PTR&} input_cloud_ptr
             * @param {CLOUD_PTR&} filtered_cloud_ptr
             * @return {*}
             */
            bool Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) override;
            /*** 
             * @description: 设置box正方体的大小
             * @param {vector<float>} size
             * @return {*}
             */
            void SetSize(std::vector<float> size);
            /*** 
             * @description: 设置box中心位姿
             * @param {vector<float>} origin
             * @return {*}
             */    
            void SetOrigin(std::vector<float> origin);
            /*** 
             * @description: 获取box的边缘
             * (x1, y1)
             *              *
             *                *
             *                   (x2,y2)
             * @return {*}
             */
            std::vector<float> GetEdge();
        private:
            void CalculateEdge();
        private:
            pcl::CropBox<PointType> pcl_box_filter_;

            std::vector<float> origin_;
            std::vector<float> size_;
            std::vector<float> edge_;
    };
}


