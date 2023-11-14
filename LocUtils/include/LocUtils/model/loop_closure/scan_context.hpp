/*
 * @Date: 2023-10-31 10:53:14
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-11-03 16:01:11
 * @FilePath: /loc_lib/LocUtils/include/LocUtils/model/loop_closure/scan_context.hpp
 */

#ifndef LOCUTILS_MODEL_LOOPCLOSURE_SC
#define LOCUTILS_MODEL_LOOPCLOSURE_SC

#pragma once

#include <LocUtils/common/eigen_types.h>
#include <LocUtils/common/point_types.h>
#include <LocUtils/common/math_utils.h>

#include <LocUtils/tools/tic_toc.hpp>

#include <algorithm>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


namespace LocUtils
{
    struct ScanContextOptions
    {
        int pc_num_ring {20}; //每帧的ring数目 20 in the original paper (IROS 18)
        int pc_num_sector {60}; //每帧的sector数目 60 in the original paper (IROS 18)
        double pc_max_radius {80}; //最大点云半径 80 meter max in the original paper (IROS 18)
        double lidar_height {2.0}; // lidar距离地面的高度（保证每个点大于0）

        bool show_ui_flag {false}; //是否显示画面
    };
    class ScanContext
    {
    private:
        ScanContextOptions sc_option_;

        double pc_unit_sector_rangle;
        double pc_unit_ring_gap; 

        /**
         * 普通点云生成SC 
         * @param {CloudPtr &} scan_down
         * @return {*}
         */
        MatXd MakeScanContext(CloudPtr & scan_down);

        /**
         * 全量点云生成SC
         * @param {FullCloudPtr &} scan_down
         * @return {*}
         */
        MatXd MakeScanContext(FullCloudPtr & scan_down);

        /**
         * 可视化sc
         * @param {MatXd} sc
         * @return {*}
         */        
        void ShowScanContext(MatXd sc);

    public:
        ScanContext(ScanContextOptions options);
        ~ScanContext();

        /*
            公有变量
        */


        /**
         * 接口函数，保存SC和对应的Key，普通点云
         * @param {CloudPtr &} _scan_down 普通点云
         * @return {*}
         */		
        void MakeAndSaveScancontextAndKeys( CloudPtr & scan_down );

        /**
         * 接口函数，保存SC和对应的Key，全量点云
         * @param {FullCloudPtr &} _scan_down
         * @return {*}
         */		
        void MakeAndSaveScancontextAndKeys( FullCloudPtr & scan_down );

        /**
         * 接口函数，回环检测，返回最近的、匹配的关键帧I索引和相对的yaw角偏移
         * @return {std::pair<int, float>} int: nearest node index, float: relative yaw  
         */
        std::pair<int, float> DetectLoopClosureID( void );
    };
    
}

#endif