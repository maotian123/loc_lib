/*
 * @Date: 2023-10-31 10:56:08
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-11-03 19:55:13
 * @FilePath: /loc_lib/LocUtils/src/model/loop_closure/scan_context.cpp
 */

#include <LocUtils/model/loop_closure/scan_context.hpp>

namespace LocUtils
{
    ScanContext::ScanContext(ScanContextOptions options)
    :sc_option_(options)
    {
        pc_unit_sector_rangle = 360.0 / double(options.pc_num_sector);
        pc_unit_ring_gap = options.pc_max_radius / double(options.pc_num_ring);

        // std::cout << "ScanContextOptions.show_ui_flag " << sc_option_.show_ui_flag << std::endl;
    }

    ScanContext::~ScanContext()
    {
    }

    void ScanContext::ShowScanContext(MatXd sc)
    {
        static bool windows_init_flag = false;
        if(!windows_init_flag)
        {
            cv::namedWindow("ScanContext_image", 0);
            cv::resizeWindow("ScanContext_image", sc_option_.pc_num_sector * 10, sc_option_.pc_num_ring * 10);
        }
        cv::Mat matrix_cv(cv::Size(sc_option_.pc_num_sector, sc_option_.pc_num_ring), CV_64FC1);
        // cv::Mat matrix_cv;
        cv::eigen2cv(sc, matrix_cv);

        double min, max;
        cv::minMaxIdx(matrix_cv, &min, &max);

        // 归一化到0-1
        // cv::normalize(matrix_cv, matrix_cv, 1, 0, CV_MINMAX);
        // 转换为8UC1
        matrix_cv.convertTo(matrix_cv, CV_8UC1, 255);

        // 转换成伪彩深度图
        cv::applyColorMap(matrix_cv, matrix_cv, cv::COLORMAP_JET);

        cv::imshow("ScanContext_image", matrix_cv);
        cv::waitKey(1);
    }

    MatXd ScanContext::MakeScanContext(CloudPtr & scan_down)
    {
        TicToc making_dec;
        
        const int NO_POINT = -1000;
        MatXd desc = NO_POINT * MatXd::Ones(sc_option_.pc_num_ring, sc_option_.pc_num_sector);   

        PointType pt;

        float azim_angle, azim_range; // wihtin 2d plane
        int ring_index, sector_index;

        for(auto &iter : scan_down->points)
        {
            pt.x = iter.x;
            pt.y = iter.y;
            pt.z = iter.z + sc_option_.lidar_height;

            // convert xyz to ring,sector
            azim_range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            azim_angle = math::xy2theta(pt.x, pt.y);

            // sc原版
            // 计算当前point属于哪个ring
            // 实际上是限制ring_index大于等于1，小于等于 pc_num_ring（20）
            ring_index = std::max( \
                std::min(sc_option_.pc_num_ring, \
                    int(std::ceil((azim_range / sc_option_.pc_max_radius) * sc_option_.pc_num_ring))), \
                1 \
            );
            // 计算当前point属于哪个sector
            sector_index = std::max(std::min(sc_option_.pc_num_sector, \
                int(std::ceil((azim_angle/360.0) * sc_option_.pc_num_sector))), \
                1 \
            );

            // // 自己修改的 易懂
            // ring_index = int(std::ceil((azim_range / sc_option_.pc_max_radius) * sc_option_.pc_num_ring));
            // sector_index = int(std::ceil((azim_angle/360.0) * sc_option_.pc_num_sector));
            // // 得转换成右值引用才行
            // math::limit_in_range<int, int>(std::move(ring_index), 1, std::move(sc_option_.pc_num_ring));
            // math::limit_in_range<int, int>(std::move(sector_index), 1, std::move(sc_option_.pc_num_sector));

            // 找到每一个bin中最大的z值 减1是因为矩阵索引从0开始的 ring_index和sector_index都是从1开始的
            if(desc(ring_index - 1, sector_index - 1) < pt.z)
                desc(ring_index - 1, sector_index - 1) = pt.z;

        }

        // reset no points to zero (for cosine dist later)
        // 将所有的-1000都重置为0，方便后续操作
        for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        {
            for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            {
                if( desc(row_idx, col_idx) == NO_POINT )
                    desc(row_idx, col_idx) = 0;
            }
        }

        // 如果可视化标志为true,显示
        if(sc_option_.show_ui_flag)
        {
            ShowScanContext(desc);
        }

        return desc;     
    }

    MatXd ScanContext::MakeScanContext(FullCloudPtr & scan_down)
    {
        TicToc making_dec;
        
        const int NO_POINT = -1000;
        MatXd desc = NO_POINT * MatXd::Ones(sc_option_.pc_num_ring, sc_option_.pc_num_sector);

        FullPointType pt;

        float azim_angle, azim_range; // wihtin 2d plane
        int ring_index, sector_index;

        for(auto &iter : scan_down->points)
        {
            pt.x = iter.x;
            pt.y = iter.y;
            pt.z = iter.z + sc_option_.lidar_height;

            // convert xyz to ring,sector
            azim_range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            azim_angle = math::xy2theta(pt.x, pt.y);

            // sc原版
            // 计算当前point属于哪个ring
            // 实际上是限制ring_index大于等于1，小于等于 pc_num_ring（20）
            ring_index = std::max( \
                std::min(sc_option_.pc_num_ring, \
                    int(std::ceil((azim_range / sc_option_.pc_max_radius) * sc_option_.pc_num_ring))), \
                1 \
            );
            // 计算当前point属于哪个sector
            sector_index = std::max(std::min(sc_option_.pc_num_sector, \
                int(std::ceil((azim_angle/360.0) * sc_option_.pc_num_sector))), \
                1 \
            );

            // // 自己修改的 易懂
            // ring_index = int(std::ceil((azim_range / sc_option_.pc_max_radius) * sc_option_.pc_num_ring));
            // sector_index = int(std::ceil((azim_angle/360.0) * sc_option_.pc_num_sector));
            // // 得转换成右值引用才行
            // math::limit_in_range<int, int>(std::move(ring_index), 1, std::move(sc_option_.pc_num_ring));
            // math::limit_in_range<int, int>(std::move(sector_index), 1, std::move(sc_option_.pc_num_sector));

            // 找到每一个bin中最大的z值 减1是因为矩阵索引从0开始的 ring_index和sector_index都是从1开始的
            if(desc(ring_index - 1, sector_index - 1) < pt.z)
                desc(ring_index - 1, sector_index - 1) = pt.z;

        }

        // reset no points to zero (for cosine dist later)
        // 将所有的-1000都重置为0，方便后续操作
        for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        {
            for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            {
                if( desc(row_idx, col_idx) == NO_POINT )
                    desc(row_idx, col_idx) = 0;
            }
        }

        // 如果可视化标志为true,显示
        if(sc_option_.show_ui_flag)
        {
            ShowScanContext(desc);
        }

        return desc;
    }

    void ScanContext::MakeAndSaveScancontextAndKeys( CloudPtr & _scan_down )
    {

    }

    void ScanContext::MakeAndSaveScancontextAndKeys( FullCloudPtr & _scan_down )
    {
        MatXd sc = MakeScanContext(_scan_down); // v1 
    }

    std::pair<int, float> ScanContext::DetectLoopClosureID( void )
    {

    }
}