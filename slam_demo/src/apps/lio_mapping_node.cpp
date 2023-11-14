//
// Created by Tian on 2023/10/26.
//
#include "mapping/3d/lio_mapping_flow.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <std_msgs/Int8.h>

using namespace LocUtils;

static std::atomic<bool> thread_enable_ {true};
static std::atomic<bool> save_map_flag_ {false};
static std::atomic<bool> save_keyframe_poses_flag_ {false};
static std::atomic<bool> save_gnss_pose_flag_ {false};

static std::shared_ptr<LioMappingFlow> lio_mapping_flow_ptr_;
void LocSubCmdCallback(const std_msgs::Int8::ConstPtr &msg)
{
    if(msg->data == 0)
    {
        save_map_flag_ = true;
        save_keyframe_poses_flag_ = true;
        save_gnss_pose_flag_ = true;
    }
}

void SaveMapThread()
{
    while(thread_enable_)
    {
        std::chrono::microseconds sleepDuration(500);
        std::this_thread::sleep_for(sleepDuration);

        if(!save_map_flag_)
            continue;

        if(lio_mapping_flow_ptr_->SaveGlobalMap())
        {
            save_map_flag_ = false;
        }

    }
}

void SaveKeyFramePoseThread()
{
    while(thread_enable_)
    {
        std::chrono::microseconds sleepDuration(500);
        std::this_thread::sleep_for(sleepDuration);

        if(!save_keyframe_poses_flag_)
            continue;

        if(lio_mapping_flow_ptr_->SaveAllKeyframePoses())
        {
            save_keyframe_poses_flag_ = false;
        }
        
    }
}

void SaveGnssPoseThread()
{
    while(thread_enable_)
    {
        std::chrono::microseconds sleepDuration(500);
        std::this_thread::sleep_for(sleepDuration);

        if(!save_gnss_pose_flag_)
            continue;

        if(lio_mapping_flow_ptr_->SaveAllGnssPoses())
        {
            save_gnss_pose_flag_ = false;
        }
        
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lio_mapping_node");
    ros::NodeHandle nh;
   
    std::string yaml_path {"/root/workspace/loc_lib/slam_demo/config/slam.yaml"};
    // std::string yaml_path {"/root/loc_lib/slam_demo/config/slam.yaml"};
    lio_mapping_flow_ptr_ = std::make_shared<LioMappingFlow>(nh, yaml_path);
    ros::Subscriber cmd_sub = nh.subscribe("loc_cmd_sub", 100, LocSubCmdCallback);
    std::thread save_map_thread(SaveMapThread);
    // std::thread save_keyframe_thread(SaveKeyFramePoseThread);
    // std::thread save_gnss_thread(SaveGnssPoseThread);
    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        lio_mapping_flow_ptr_->Run();
        rate.sleep();
    }

    thread_enable_ = false;
    save_map_thread.join();
    // save_keyframe_thread.join();
    // save_gnss_thread.join();
}