/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-10-12 14:51:36
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-27 20:15:48

*/
#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include "LocUtils/common/eigen_types.h"
#include "geometry_msgs/PoseStamped.h"
#include "LocUtils/common/pose_data.hpp"

namespace LocUtils
{

class SavePose
{
public:
    enum class SavePoseMethod {
        KITTI, //KITTI格式
        TUM //TUM格式
    }; 
    SavePose(const SavePoseMethod& method, const std::string& pose_file_path);
    ~SavePose();

    /************************************************/
    //保存单个位姿到文件（ROS）
    void SavePoseToFile(const geometry_msgs::PoseStamped& pose_stamped);

    /**
     * @description: 保存单个位姿到文件（非ROS)
     * @param {SE3&} pose
     * @param {double&} time
     * @return {*}
     */    
    void SavePoseToFile(const SE3& pose, double& time);

    /**
     * @description: 保存单个位姿到文件（非ROS)
     * @param {PoseData&} pose
     * @return {*}
     */    
    void SavePoseToFile(PoseData& pose);

    /************************************************/
    //保存一组vector到文件（ROS）
    void SaveAllPoseToFile(const std::deque<geometry_msgs::PoseStamped>& pose_stamped_vec);


    /**
     * @description: 保存一组vector到文件（非ROS）
     * @param {std::deque<SE3>& poses&} poses
     * @param {std::vector<double>&} times
     * @return {*}
     */    
    void SaveAllPoseToFile(const std::deque<SE3>& poses, std::vector<double>& times);
    
    /**
     * @description: 保存一组vector到文件（非ROS）
     * @param {std::vector<PoseData>&} poses
     * @return {*}
     */    
    void SaveAllPoseToFile(std::deque<PoseData>& poses);

    /************************************************/
private:
    std::string pose_file_path_;
    std::ofstream pose_file_;
    SavePoseMethod method_;
    void PrintSavePoseMethodUse();  

};





}