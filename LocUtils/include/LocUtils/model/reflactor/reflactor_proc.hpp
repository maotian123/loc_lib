//
// Created by Tian on 2023/09/06.
//
#include "LocUtils/common/lidar_utils.h"

namespace LocUtils
{
    struct ReflactorOption {
        float _0_1_0_5_INT  {105.0f};
        float _0_5_2_INT    {70.0f};
        float _2_4_INT      {50.0f};
        float _4_6_INT      {40.0f};
        float _6_INT        {30.0f};
        float reflectiveTargetNomalWidth    {0.05f};
        float reflectiveTargetNomalWidthMin {0.025f};
        float reflectiveTargetNomalWidthMax {0.085f};
        float matching_angle_min {10.0f};                  //三角形最小角度
        float matching_angle_max {170.0f};                 //三角形最大角度
        float matching_error {0.03f};                    //计算得到的结果判断并输出的阈值，三角形边长
        float matching_angle_error {0.03f};              //计算得到的结果判断并输出的阈值，三角形顶角弧度
        float recognition_distance {6.0f};               //雷达最远探测距离
    };
    class ReflactorProcess
    {
        public:
            ReflactorProcess();
            ReflactorProcess(ReflactorOption option);
            
            void SetOption(const ReflactorOption &option);

            void AddScan(const Scan2d &scan);
        private:
            ReflactorOption option_;
    };
}