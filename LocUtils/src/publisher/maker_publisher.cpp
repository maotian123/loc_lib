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

    void MarkerPublisher::SetupMarker(visualization_msgs::Marker &m, int id,
                                      float x, float y, float z,
                                      const Vec3f &rgb)
    {
        m.header.frame_id = frame_id_;
        m.header.stamp = ros::Time();
        m.ns = topic_name_;
        m.id = id;
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.25;
        m.scale.y = 0.25;
        m.scale.z = 0.5;
        m.color.r = rgb.x();
        m.color.g = rgb.y();
        m.color.b = rgb.z();
        m.color.a = 0.5;
    }

    void MarkerPublisher::Publish(const std::vector<Vec2f> &marker)
    {
        Publish(marker, Vec3f(0.0f, 0.0f, 1.0f));
    }

    void MarkerPublisher::Publish(const std::vector<Vec3f> &marker)
    {
        Publish(marker, Vec3f(0.0f, 0.0f, 1.0f));
    }

    void MarkerPublisher::Publish(const std::vector<Vec2f> &marker, const Vec3f &rgb)
    {
        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            SetupMarker(maker_.markers[i], i, marker[i].x(), marker[i].y(), 0.0f, rgb);
        }
        publisher_.publish(maker_);
    }

    void MarkerPublisher::Publish(const std::vector<Vec3f> &marker, const Vec3f &rgb)
    {
        maker_.markers.resize(marker.size());
        for(int i {0}; i < marker.size(); ++i)
        {
            SetupMarker(maker_.markers[i], i, marker[i].x(), marker[i].y(), marker[i].z(), rgb);
        }
        publisher_.publish(maker_);
    }
}
