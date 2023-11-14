/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-07-06 17:41:54
 * @LastEditors: lxy
 * @LastEditTime: 2023-07-06 18:19:10
 */
#ifndef LOCUTILS_MAPPING_2D_MAPPING_2D_HPP_
#define LOCUTILS_MAPPING_2D_MAPPING_2D_HPP_
#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/lidar_utils.h"
#include "LocUtils/sensor_data/scan_frame.hpp"
#include <memory>
#include <opencv2/core.hpp>

namespace LocUtils
{

class Submap;
class LoopClosing;

/**
 * 2D 激光建图的主要类
 */
class Mapping2D {
   public:
    bool Init(bool with_loop_closing = true);

    /// 单回波的scan
    bool ProcessScan(Scan2d::Ptr scan);

    /// 多回波的scan
    /// 暂时没用到
    bool ProcessScan(MultiScan2d::Ptr scan);

    /**
     * 显示全局地图
     * @param max_size 全局地图最大长宽
     * @return 全局地图图像
     */
    cv::Mat ShowGlobalMap(int max_size = 500);

   private:
    /// 判定当前帧是否为关键帧
    bool IsKeyFrame();

    void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap);
    /// 增加一个关键帧
    void AddKeyFrame();

    /// 扩展新的submap
    void ExpandSubmap();

    /// 数据成员
    size_t frame_id_ = 0;
    size_t keyframe_id_ = 0;
    size_t submap_id_ = 0;

    bool first_scan_ = true;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> last_frame_ = nullptr;
    SE2 motion_guess_;
    std::shared_ptr<Frame> last_keyframe_ = nullptr;
    std::shared_ptr<Submap> current_submap_ = nullptr;

    std::vector<std::shared_ptr<Submap>> all_submaps_;

    std::shared_ptr<LoopClosing> loop_closing_ = nullptr;  // 回环检测

    // 参数
    inline static constexpr double keyframe_pos_th_ = 0.3;              // 关键帧位移量
    inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;  // 关键帧角度量
};


    
} // namespace LocUtils










#endif