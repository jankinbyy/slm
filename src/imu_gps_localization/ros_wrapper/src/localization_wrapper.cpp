#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>
#include <iostream>
#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle &nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise", acc_noise, 1e-2);
    nh.param("gyro_noise", gyro_noise, 1e-4);
    nh.param("acc_bias_noise", acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);
    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);
    std::string log_folder;
    ros::param::get("log_folder", log_folder);
    // Log.
    file_state_.open(log_folder + "/log/state.csv");
    file_gps_.open(log_folder + "/log/gps.csv");
    Config::readConfig(log_folder);
    imu_gps_localizer_ptr_ =
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);
    dr_odo_flow_ptr = std::make_unique<DrOdoFlow>(Config::imu_T_wheel_);
    run_thread_ = std::thread(&LocalizationWrapper::RunFunc, this);
    // Subscribe topics.
    imu_sub_ = nh.subscribe(Config::imu_topic_, 10, &LocalizationWrapper::ImuCallback, this);
    wheel_sub_ = nh.subscribe(Config::wheel_topic_, 10, &LocalizationWrapper::WheelCallback, this);
    gps_position_sub_ = nh.subscribe(Config::rtk_topic_, 10, &LocalizationWrapper::GpsPositionCallback, this);
    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
    dr_path_pub_ = nh.advertise<nav_msgs::Path>("dr_path", 10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}
void LocalizationWrapper::WheelCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr) {
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();
    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x * 0.95;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;
    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;
    dr_odo_flow_ptr->ReadData(velocity_data);
}
void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x,
        imu_msg_ptr->linear_acceleration.y,
        imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
        imu_msg_ptr->angular_velocity.y,
        imu_msg_ptr->angular_velocity.z;

    ImuGpsLocalization::State fused_state;
    bool ok = false;
    if (1)
        ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    else
        ok = imu_gps_localizer_ptr_->ProcessImuData(dr_odo_flow_ptr->GetPos(), imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }
    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);
    // Log fused state.
    LogState(fused_state);
    //new
    IMUData dr_imu;
    dr_imu.time = imu_msg_ptr->header.stamp.toSec();
    dr_imu.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    dr_imu.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    dr_imu.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;
    dr_imu.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    dr_imu.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    dr_imu.angular_velocity.z = imu_msg_ptr->angular_velocity.z;
    dr_odo_flow_ptr->ReadData(dr_imu);
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->position_covariance[0] + gps_msg_ptr->position_covariance[4] > 0.005) { // RTK 噪声过大，认为是不可信的 fixme:是否存在中间的临界状态？
        LOG(WARNING) << "[GpsCallBack]: Bad gps cov message!";
        return;
    }
    if (gps_msg_ptr->status.status != 4) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }
    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
        gps_msg_ptr->longitude,
        gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    if (imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr))
        LogGps(gps_data_ptr);
}
void LocalizationWrapper::RunFunc() {
    while (1) {
        if (dr_odo_flow_ptr->Optimization()) {
            Eigen::Matrix4d now_pos = dr_odo_flow_ptr->GetPos();
            ConvertEigenToRosTopic(now_pos);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
void LocalizationWrapper::LogState(const ImuGpsLocalization::State &state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State &state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;
    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();
    ros_path_.poses.push_back(pose);
}
void LocalizationWrapper::ConvertEigenToRosTopic(const Eigen::Matrix4d &state) {
    dr_path_.header.frame_id = "world";
    dr_path_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = dr_path_.header;
    pose.pose.position.x = state(0, 3);
    pose.pose.position.y = state(1, 3);
    pose.pose.position.z = state(2, 3);
    dr_path_.poses.push_back(pose);
    dr_path_pub_.publish(dr_path_);
}