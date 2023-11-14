//
// Created by Tian on 2023/10/27.
//

#include <memory>
#include "matching/3d/lio_matching_flow.hpp"

using namespace LocUtils;

static std::shared_ptr<LioMatchingFlow> lio_matching_ptr_;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lio_matching_node");
    ros::NodeHandle nh;

    std::string yaml_path {"/root/workspace/loc_lib/slam_demo/config/slam.yaml"};
    
    lio_matching_ptr_ = std::make_shared<LioMatchingFlow>(nh, yaml_path);
    ros::Rate rate(200);

    while(ros::ok())
    {
        ros::spinOnce();
        lio_matching_ptr_->Run();
        rate.sleep();
    }
}