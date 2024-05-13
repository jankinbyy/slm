#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "nav_msgs/Odometry.h"
#include "imu_gps_localizer/imu_gps_localizer.h"
#include "dr_odo_flow.hpp"
#include "config.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle &nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);
    void WheelCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);
    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State &state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);
    void ConvertStateToRosTopic(const ImuGpsLocalization::State &state);
    void ConvertEigenToRosTopic(const Eigen::Matrix4d &state);
    void RunFunc();
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber wheel_sub_;
    ros::Publisher state_pub_;
    ros::Publisher dr_path_pub_;
    std::ofstream file_state_;
    std::ofstream file_gps_;
    nav_msgs::Path ros_path_;
    nav_msgs::Path dr_path_;
    std::thread run_thread_;
    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_; //非共享指针，shared是共享指针
    std::unique_ptr<DrOdoFlow> dr_odo_flow_ptr;
};