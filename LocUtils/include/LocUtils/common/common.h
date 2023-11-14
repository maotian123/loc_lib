/*** 
 * @Author: Tian YiHong
 * @Date: 2023-07-26 23:51:47
 * @LastEditTime: 2023-07-31 08:12:37
 * @LastEditors: Tian YiHong
 * @Description: 
 * @FilePath: /loc-utils/LocUtils/include/LocUtils/common/common.h
 */
#pragma once
#include <boost/filesystem.hpp>
#include <iostream>
/**
 * @name: 定义传感器类型 
 * @msg: 
 * @return {*}
 */
namespace LocUtils
{
    typedef enum
    {
        SENSOR_TYPE_LIDAR = 1,
        SENSOR_TYPE_IMU = 2
    }SENSOR_TYPE_E;

    static std::string GetCwd()
    {
        boost::filesystem::path exePath = boost::filesystem::read_symlink("/proc/self/exe");
        std::cout << "Executable path: " << exePath << std::endl;

        // 获取当前工作目录
        boost::filesystem::path cwdPath = boost::filesystem::current_path();
        std::cout << "Current working directory: " << cwdPath << std::endl;

        return exePath.string();
    }
    // {
    //     boost::filesystem::path exePath = boost::filesystem::read_symlink("/proc/self/exe");
    //     std::cout << "Executable path: " << exePath << std::endl;

    //     // 获取当前工作目录
    //     boost::filesystem::path cwdPath = boost::filesystem::current_path();
    //     std::cout << "Current working directory: " << cwdPath << std::endl;

    //     return exePath.string();
    // }
}
