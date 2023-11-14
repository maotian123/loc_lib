/*
 * @Descripttion: 
 * @version: 
 * @Author: lxy
 * @Date: 2023-10-12 15:08:21
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-29 17:47:22
 */
#include "LocUtils/tools/save_pose.hpp"
#include <glog/logging.h>
namespace LocUtils
{
    SavePose::SavePose(const SavePoseMethod& method, const std::string& pose_file_path)
    :method_(method),pose_file_path_(pose_file_path)
    {
        pose_file_.open(pose_file_path_);
        pose_file_.setf(std::ios::fixed);// 关闭科学计数格式
        pose_file_.precision(12);// 保留12位小数
    }

    void SavePose::PrintSavePoseMethodUse()
    {
        if(method_ == SavePoseMethod::KITTI)
        {
            LOG(INFO)<<"Use SavePoseMethod::KITTI";
        }else if(method_ == SavePoseMethod::TUM)
        {
            LOG(INFO)<<"Use SavePoseMethod::TUM";
        }else
        {
            LOG(INFO)<<"Not a valid SavePoseMethod";
        }
    }  
    SavePose::~SavePose()
    {
        pose_file_.close();
    }

    void SavePose::SavePoseToFile(const geometry_msgs::PoseStamped& pose_stamped)
    {
        if(method_ == SavePoseMethod::KITTI)
        {
            Eigen::Quaterniond q_tmp;            
            q_tmp.x() = pose_stamped.pose.orientation.x;
            q_tmp.y() = pose_stamped.pose.orientation.y;
            q_tmp.z() = pose_stamped.pose.orientation.z;
            q_tmp.w() = pose_stamped.pose.orientation.w;
            Eigen::Matrix3d R_tmp;
            R_tmp = q_tmp.normalized().toRotationMatrix();  
            Eigen::Matrix<double, 4, 4> kitti_pose;
            kitti_pose.topLeftCorner(3,3) = R_tmp;
            kitti_pose(0,3) = pose_stamped.pose.position.x;
            kitti_pose(1,3) = pose_stamped.pose.position.y;
            kitti_pose(2,3) = pose_stamped.pose.position.z;
            pose_file_ << kitti_pose(0,0) << " " << kitti_pose(0,1) << " " << kitti_pose(0,2) << " " << kitti_pose(0,3) << " "
            << kitti_pose(1,0) << " " << kitti_pose(1,1) << " " << kitti_pose(1,2) << " " << kitti_pose(1,3) << " "
            << kitti_pose(2,0) << " " << kitti_pose(2,1) << " " << kitti_pose(2,2) << " " << kitti_pose(2,3) << std::endl;
        }else if(method_ == SavePoseMethod::TUM)
        {
            pose_file_ << pose_stamped.header.stamp << " " <<  pose_stamped.pose.position.x << " " <<  pose_stamped.pose.position.y << " " <<  pose_stamped.pose.position.z << " "
                << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " " << pose_stamped.pose.orientation.z << " " << pose_stamped.pose.orientation.w <<std::endl;
                
        }else
        {
            LOG(INFO)<<"Not to save pose, use a method_ to save pose";
        }
    }

    void SavePose::SavePoseToFile(const SE3& pose, double& time)
    {
        if (method_ == SavePoseMethod::KITTI)
        {
            Eigen::Matrix<double, 4, 4> kitti_pose = SE3TOMAT4<double>(pose);
            pose_file_ << kitti_pose(0, 0) << " " << kitti_pose(0, 1) << " " << kitti_pose(0, 2) << " " << kitti_pose(0, 3) << " "
                << kitti_pose(1, 0) << " " << kitti_pose(1, 1) << " " << kitti_pose(1, 2) << " " << kitti_pose(1, 3) << " "
                << kitti_pose(2, 0) << " " << kitti_pose(2, 1) << " " << kitti_pose(2, 2) << " " << kitti_pose(2, 3) << std::endl;
        }
        else if (method_ == SavePoseMethod::TUM)
        {
            Vec3d trans = pose.translation();
            Eigen::Quaterniond q(pose.rotationMatrix());
            q.normalize();
            pose_file_ << time << " " << trans.x() << " " << trans.y() << " " << trans.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        else
        {
            LOG(INFO) << "Not to save pose, use a method_ to save pose";
        }
    }

    void SavePose::SavePoseToFile(PoseData& pose)
    {
        if (method_ == SavePoseMethod::KITTI)
        {
            Eigen::Matrix4d kitti_pose = pose.pose_.cast<double>(); 
            pose_file_ << kitti_pose(0, 0) << " " << kitti_pose(0, 1) << " " << kitti_pose(0, 2) << " " << kitti_pose(0, 3) << " "
                << kitti_pose(1, 0) << " " << kitti_pose(1, 1) << " " << kitti_pose(1, 2) << " " << kitti_pose(1, 3) << " "
                << kitti_pose(2, 0) << " " << kitti_pose(2, 1) << " " << kitti_pose(2, 2) << " " << kitti_pose(2, 3) << std::endl;
        }
        else if (method_ == SavePoseMethod::TUM)
        {
            Eigen::Quaterniond q = pose.GetQuaternion().cast<double>();
            q.normalize();
            pose_file_ << pose.time_ << " " << pose.pose_(0, 3) << " " << pose.pose_(1, 3) << " " << pose.pose_(2, 3) << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        else
        {
            LOG(INFO) << "Not to save pose, use a method_ to save pose";
        }
    }

    void SavePose::SaveAllPoseToFile(const std::deque<geometry_msgs::PoseStamped>& pose_stamped_vec)
    {
        for(auto ite = pose_stamped_vec.begin();ite!=pose_stamped_vec.end();ite++)
        {

            geometry_msgs::PoseStamped pose_stamped = *ite;

            if(method_ == SavePoseMethod::KITTI)
            {
                Eigen::Quaterniond q_tmp;            
                q_tmp.x() = pose_stamped.pose.orientation.x;
                q_tmp.y() = pose_stamped.pose.orientation.y;
                q_tmp.z() = pose_stamped.pose.orientation.z;
                q_tmp.w() = pose_stamped.pose.orientation.w;
                Eigen::Matrix3d R_tmp;
                R_tmp = q_tmp.normalized().toRotationMatrix();  
                Eigen::Matrix<double, 4, 4> kitti_pose;
                kitti_pose.topLeftCorner(3,3) = R_tmp;
                kitti_pose(0,3) = pose_stamped.pose.position.x;
                kitti_pose(1,3) = pose_stamped.pose.position.y;
                kitti_pose(2,3) = pose_stamped.pose.position.z;
                pose_file_ << kitti_pose(0,0) << " " << kitti_pose(0,1) << " " << kitti_pose(0,2) << " " << kitti_pose(0,3) << " "
                << kitti_pose(1,0) << " " << kitti_pose(1,1) << " " << kitti_pose(1,2) << " " << kitti_pose(1,3) << " "
                << kitti_pose(2,0) << " " << kitti_pose(2,1) << " " << kitti_pose(2,2) << " " << kitti_pose(2,3) << std::endl;
            }else if(method_ == SavePoseMethod::TUM)
            {
                pose_file_ << pose_stamped.header.stamp << " " <<  pose_stamped.pose.position.x << " " <<  pose_stamped.pose.position.y << " " <<  pose_stamped.pose.position.z << " "
                    << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " " << pose_stamped.pose.orientation.z << " " << pose_stamped.pose.orientation.w <<std::endl;
                    
            }else
            {
                LOG(INFO)<<"Not to save pose, use a method_ to save pose";
            }         
        } 
    }

    void SavePose::SaveAllPoseToFile(const std::deque<SE3>& poses, std::vector<double>& times)
    {
        for(auto &p : poses)
        {
            if (method_ == SavePoseMethod::KITTI)
            {
                Eigen::Matrix<double, 4, 4> kitti_pose = SE3TOMAT4(p);
                pose_file_ << kitti_pose(0, 0) << " " << kitti_pose(0, 1) << " " << kitti_pose(0, 2) << " " << kitti_pose(0, 3) << " "
                    << kitti_pose(1, 0) << " " << kitti_pose(1, 1) << " " << kitti_pose(1, 2) << " " << kitti_pose(1, 3) << " "
                    << kitti_pose(2, 0) << " " << kitti_pose(2, 1) << " " << kitti_pose(2, 2) << " " << kitti_pose(2, 3) << std::endl;
            }
            else if (method_ == SavePoseMethod::TUM)
            {
                Vec3d trans = p.translation();
                Eigen::Quaterniond q(p.rotationMatrix());
                q.normalize();
                pose_file_ << time << " " << trans.x() << " " << trans.y() << " " << trans.z() << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            }
            else
            {
                LOG(INFO) << "Not to save pose, use a method_ to save pose";
                return;
            }
        }
    }

    void SavePose::SaveAllPoseToFile(std::deque<PoseData>& poses)
    {
        for (auto& p : poses)
        {
            if (method_ == SavePoseMethod::KITTI)
            {
                Eigen::Matrix4d kitti_pose = p.pose_.cast<double>();
                pose_file_ << kitti_pose(0, 0) << " " << kitti_pose(0, 1) << " " << kitti_pose(0, 2) << " " << kitti_pose(0, 3) << " "
                    << kitti_pose(1, 0) << " " << kitti_pose(1, 1) << " " << kitti_pose(1, 2) << " " << kitti_pose(1, 3) << " "
                    << kitti_pose(2, 0) << " " << kitti_pose(2, 1) << " " << kitti_pose(2, 2) << " " << kitti_pose(2, 3) << std::endl;
            }
            else if (method_ == SavePoseMethod::TUM)
            {
                Eigen::Quaterniond q = p.GetQuaternion().cast<double>();
                q.normalize();
                pose_file_ << p.time_ << " " << p.pose_(0, 3) << " " << p.pose_(1, 3) << " " << p.pose_(2, 3) << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            }
            else
            {
                LOG(INFO) << "Not to save pose, use a method_ to save pose";
                return;
            }
        }
    }
}