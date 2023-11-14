/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-07-05 16:37:47
 * @LastEditors: lxy
 * @LastEditTime: 2023-07-06 11:54:56
 */
#ifndef LOCUTILS_MODEL_LIKELIHOODFIELD_HPP_
#define LOCUTILS_MODEL_LIKELIHOODFIELD_HPP_
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "LocUtils/common/eigen_types.h"
#include "LocUtils/common/lidar_utils.h"

namespace LocUtils
{


class LikelihoodField {
   public:
    /// 2D 场的模板，在设置target scan或map的时候生成
    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    LikelihoodField() { BuildModel(); }

    /// 增加一个2D的目标scan
    void SetTargetScan(Scan2d::Ptr scan);

    /// 设置被配准的那个scan
    void SetSourceScan(Scan2d::Ptr scan);

    /// 从占据栅格地图生成一个似然场地图
    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);

    /// 使用高斯牛顿法配准
    bool AlignGaussNewton(SE2& init_pose);

    /**
     * 使用g2o配准
     * @param init_pose 初始位姿 NOTE 使用submap时，给定相对于该submap的位姿，估计结果也是针对于这个submap的位姿
     * @return
     */
    bool AlignG2O(SE2& init_pose);

    /// 获取场函数，转换为RGB图像
    cv::Mat GetFieldImage();

    bool HasOutsidePoints() const { return has_outside_pts_; }

    void SetPose(const SE2& pose) { pose_ = pose; }

    void DebugLikeHood(Scan2d::Ptr curscan,Scan2d::Ptr prescan, const SE2& pose);
   private:
    void BuildModel();
    void DebugImageShow(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size = 800,
                     float resolution = 20.0);
    SE2 pose_;  // T_W_S
    Scan2d::Ptr target_ = nullptr;
    Scan2d::Ptr source_ = nullptr;

    std::vector<ModelPoint> model_;  // 2D 模板
    cv::Mat field_;                  // 场函数
    bool has_outside_pts_ = false;   // 是否含有出了这个场的点

    // 参数配置
    inline static const float resolution_ = 20;  // 每米多少个像素
};






    
} // namespace LocUtils







#endif
