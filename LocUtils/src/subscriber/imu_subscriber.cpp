#include "LocUtils/subscriber/imu_subscriber.hpp"
#include "glog/logging.h"

namespace LocUtils{
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) 
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    buff_mutex_.lock();

    // convert ROS IMU to GeographicLib compatible GNSS message:
    IMU imu_data;
    imu_data.timestamp_ = imu_msg_ptr->header.stamp.toSec();

    imu_data.acce_[0] = imu_msg_ptr->linear_acceleration.x;
    imu_data.acce_[1] = imu_msg_ptr->linear_acceleration.y;
    imu_data.acce_[2] = imu_msg_ptr->linear_acceleration.z;

    imu_data.gyro_[0] = imu_msg_ptr->angular_velocity.x;
    imu_data.gyro_[1] = imu_msg_ptr->angular_velocity.y;
    imu_data.gyro_[2] = imu_msg_ptr->angular_velocity.z;

    imu_data.ori.x() = imu_msg_ptr->orientation.x;
    imu_data.ori.y() = imu_msg_ptr->orientation.y;
    imu_data.ori.z() = imu_msg_ptr->orientation.z;
    imu_data.ori.w() = imu_msg_ptr->orientation.w;
    
    // add new message to buffer:
    new_imu_data_.push_back(imu_data);
    
    buff_mutex_.unlock();
}

void IMUSubscriber::ParseData(std::deque<IMU>& imu_data_buff) {
    buff_mutex_.lock();
    // pipe all available measurements to output buffer:
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }

    buff_mutex_.unlock();
}
}