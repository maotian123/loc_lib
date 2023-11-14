#pragma once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

#include <LocUtils/common/common.h>
struct slam_2d
{
    std::string scan_topic;
    std::string map_topic;
};

struct slam_3d
{
    std::string cloud_topic;
    std::string map_topic;
};

struct slam_params
{
    slam_2d param_2d;
    slam_3d param_3d;
};

class SlamParams
{
    public:
    
        SlamParams(const std::string &params);
        slam_params GetParams() const;

    private:
        slam_params param_;
};
