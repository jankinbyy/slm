#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>
#include <iostream>
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d &I_p_Gps) :
    initialized_(false) {
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise,
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State *fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    // Predict.
    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
    //std::cout << "g pi is:" << state_.G_p_I << std::endl;
    *fused_state = state_;
    return true;
}

bool ImuGpsLocalizer::ProcessImuData(const Eigen::Matrix4d delta_p, const ImuDataPtr imu_data_ptr, State *fused_state) {
    // Predict.
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    {
    }
    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
    //std::cout << "g pi is:" << state_.G_p_I << std::endl;
    *fused_state = state_;
    return true;
}

double last_lla = 0.0;
bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        std::cout << "init lla:" << init_lla_ << std::endl;
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }
    if (last_lla != gps_data_ptr->lla[0]) {
        // Update.
        //std::cout << "update" << std::endl;
        gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);
        last_lla = gps_data_ptr->lla[0];
        return true;
    }
    return false;
}

} // namespace ImuGpsLocalization