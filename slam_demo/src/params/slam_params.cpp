#include "params/slam_params.hpp"

SlamParams::SlamParams(const std::string &params)
{
    // LocUtils::GetCwd();
    LocUtils::GetCwd();
}

slam_params SlamParams::GetParams() const
{
    return param_;
}