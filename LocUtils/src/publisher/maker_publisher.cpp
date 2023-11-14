#include "LocUtils/publisher/marker_publisher.hpp"

namespace LocUtils
{
    MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, std::string topic_name,
                                    std::string frame_id,
                                    size_t buff_size)
                                    : nh_(nh)
                                    ,frame_id_(frame_id)
                                    ,topic_name_(topic_name)
    {
        publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_, buff_size);
    }

    void MarkerPublisher::Publish(const std::vector<Vec2f> &marker)
    {   

        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            maker_.markers[i].header.frame_id = frame_id_;
            maker_.markers[i].header.stamp = ros::Time();
            maker_.markers[i].ns = topic_name_;
            maker_.markers[i].id = i;
            maker_.markers[i].type = visualization_msgs::Marker::CYLINDER; // Marker类型为圆柱型
            maker_.markers[i].action = visualization_msgs::Marker::ADD;
            maker_.markers[i].pose.position.x = marker[i].x();
            maker_.markers[i].pose.position.y = marker[i].y();
            maker_.markers[i].pose.position.z = 0;
            maker_.markers[i].pose.orientation.x = 0.0;
            maker_.markers[i].pose.orientation.y = 0.0;
            maker_.markers[i].pose.orientation.z = 0.0;
            maker_.markers[i].pose.orientation.w = 1.0;
            maker_.markers[i].scale.x = 0.25;
            maker_.markers[i].scale.y = 0.25;
            maker_.markers[i].scale.z = 0.5; // 指定标记的比例。对于基本形状，所有方向上的 1 表示边长为 1 米。
            maker_.markers[i].color.r = 0.0;
            maker_.markers[i].color.g = 0.0;
            maker_.markers[i].color.b = 1.0; // 标记的颜色被指定为 std_msgs/ColorRGBA。每个成员应介于 0 和 1 之间。
            maker_.markers[i].color.a = 0.5; // alpha (a) 值为 0 表示完全透明（不可见），1 表示完全不透明。
        }
        publisher_.publish(maker_);
        
    }

    void MarkerPublisher::Publish(const std::vector<Vec3f> &marker)
    {   

        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            maker_.markers[i].header.frame_id = frame_id_;
            maker_.markers[i].header.stamp = ros::Time();
            maker_.markers[i].ns = topic_name_;
            maker_.markers[i].id = i;
            maker_.markers[i].type = visualization_msgs::Marker::CYLINDER; // Marker类型为圆柱型
            maker_.markers[i].action = visualization_msgs::Marker::ADD;
            maker_.markers[i].pose.position.x = marker[i].x();
            maker_.markers[i].pose.position.y = marker[i].y();
            maker_.markers[i].pose.position.z = marker[i].z();
            maker_.markers[i].pose.orientation.x = 0.0;
            maker_.markers[i].pose.orientation.y = 0.0;
            maker_.markers[i].pose.orientation.z = 0.0;
            maker_.markers[i].pose.orientation.w = 1.0;
            maker_.markers[i].scale.x = 0.25;
            maker_.markers[i].scale.y = 0.25;
            maker_.markers[i].scale.z = 0.5; // 指定标记的比例。对于基本形状，所有方向上的 1 表示边长为 1 米。
            maker_.markers[i].color.r = 0.0;
            maker_.markers[i].color.g = 0.0;
            maker_.markers[i].color.b = 1.0; // 标记的颜色被指定为 std_msgs/ColorRGBA。每个成员应介于 0 和 1 之间。
            maker_.markers[i].color.a = 0.5; // alpha (a) 值为 0 表示完全透明（不可见），1 表示完全不透明。
        }
        publisher_.publish(maker_);
        
    }

    void MarkerPublisher::Publish(const std::vector<Vec2f> &marker, const Vec3f &rgb)
    {
        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            maker_.markers[i].header.frame_id = frame_id_;
            maker_.markers[i].header.stamp = ros::Time();
            maker_.markers[i].ns = topic_name_;
            maker_.markers[i].id = i;
            maker_.markers[i].type = visualization_msgs::Marker::CYLINDER; // Marker类型为圆柱型
            maker_.markers[i].action = visualization_msgs::Marker::ADD;
            maker_.markers[i].pose.position.x = marker[i].x();
            maker_.markers[i].pose.position.y = marker[i].y();
            maker_.markers[i].pose.position.z = 0;
            maker_.markers[i].pose.orientation.x = 0.0;
            maker_.markers[i].pose.orientation.y = 0.0;
            maker_.markers[i].pose.orientation.z = 0.0;
            maker_.markers[i].pose.orientation.w = 1.0;
            maker_.markers[i].scale.x = 0.25;
            maker_.markers[i].scale.y = 0.25;
            maker_.markers[i].scale.z = 0.5; // 指定标记的比例。对于基本形状，所有方向上的 1 表示边长为 1 米。
            maker_.markers[i].color.r = rgb.x();
            maker_.markers[i].color.g = rgb.y();
            maker_.markers[i].color.b = rgb.z(); // 标记的颜色被指定为 std_msgs/ColorRGBA。每个成员应介于 0 和 1 之间。
            maker_.markers[i].color.a = 0.5; // alpha (a) 值为 0 表示完全透明（不可见），1 表示完全不透明。
        }
        publisher_.publish(maker_);
    }

    void MarkerPublisher::Publish(const std::vector<Vec3f> &marker, const Vec3f &rgb)
    {
        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            maker_.markers[i].header.frame_id = frame_id_;
            maker_.markers[i].header.stamp = ros::Time();
            maker_.markers[i].ns = topic_name_;
            maker_.markers[i].id = i;
            maker_.markers[i].type = visualization_msgs::Marker::CYLINDER; // Marker类型为圆柱型
            maker_.markers[i].action = visualization_msgs::Marker::ADD;
            maker_.markers[i].pose.position.x = marker[i].x();
            maker_.markers[i].pose.position.y = marker[i].y();
            maker_.markers[i].pose.position.z = marker[i].z();
            maker_.markers[i].pose.orientation.x = 0.0;
            maker_.markers[i].pose.orientation.y = 0.0;
            maker_.markers[i].pose.orientation.z = 0.0;
            maker_.markers[i].pose.orientation.w = 1.0;
            maker_.markers[i].scale.x = 0.25;
            maker_.markers[i].scale.y = 0.25;
            maker_.markers[i].scale.z = 0.5; // 指定标记的比例。对于基本形状，所有方向上的 1 表示边长为 1 米。
            maker_.markers[i].color.r = rgb.x();
            maker_.markers[i].color.g = rgb.y();
            maker_.markers[i].color.b = rgb.z(); // 标记的颜色被指定为 std_msgs/ColorRGBA。每个成员应介于 0 和 1 之间。
            maker_.markers[i].color.a = 0.5; // alpha (a) 值为 0 表示完全透明（不可见），1 表示完全不透明。
        }
        publisher_.publish(maker_);
    }
}