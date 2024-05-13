#include "dr_odo_flow.hpp"

using namespace std;

bool DrOdoFlow::Optimization() {
    bool ret = false;
    if (reset_flag_dro) {
        clear_state_dro();
    } else if (is_start) {
        // ReadData();
        while (!raw_imu_.empty()) {
            updateDrOdo();
            ret = true;
        }
    }
    return ret;
}
void DrOdoFlow::clear_state_dro() { //  清空 DR_ODO 的所有状态，包括滤波器重置以及buffer清空

    orientation_estimator->clear_state_ori();
    pose_mtx_.lock();
    Twi = Eigen::Matrix4d::Identity();
    pose_mtx_.unlock();
    raw_imu_.clear();
    unsynced_imu_.clear();
    unsynced_velocity_.clear();
    dr_buffer.clear();
}

void DrOdoFlow::Start() {
    is_start = true;
}

void DrOdoFlow::Stop() {
    is_start = false;
}

void DrOdoFlow::ReadData(IMUData &input_imu, VelocityData &in_wheel) //放入数据
{
    raw_imu_.push_back(input_imu);
    if (abs(in_wheel.linear_velocity.x) > 0.0 || abs(in_wheel.linear_velocity.y) > 0.0 || abs(in_wheel.linear_velocity.z) > 0.0)
        nonZeroLast = in_wheel;
    unsynced_velocity_.push_back(in_wheel);
}
void DrOdoFlow::ReadData(IMUData &input_imu) {
    raw_imu_.push_back(input_imu);
}
void DrOdoFlow::ReadData(VelocityData &in_wheel) {
    if (abs(in_wheel.linear_velocity.x) > 0.0 || abs(in_wheel.linear_velocity.y) > 0.0 || abs(in_wheel.linear_velocity.z) > 0.0)
        nonZeroLast = in_wheel;
    unsynced_velocity_.push_back(in_wheel);
}
void DrOdoFlow::updateDrOdo() {
    while (!raw_imu_.empty()) {
        IMUData imu_in = raw_imu_.front();
        IMUData imuSelf = preIntegrate(imu_in, orientation_estimator);
        unsynced_imu_.push_back(imuSelf);
        raw_imu_.pop_front();
    }
    while (unsynced_imu_.size() > 1) {
        IMUData imu_front = unsynced_imu_.at(0);
        IMUData imu_back = unsynced_imu_.at(1);
        if (imu_front.time == imu_back.time) {
            unsynced_imu_.pop_front();
            continue;
        }

        Eigen::Matrix4d T_prev_cur_imu = Fusing_IMU_VelOdom(imu_front, imu_back);
        //std::cout << "fusion pos:" << T_prev_cur_imu(0, 3) << "," << T_prev_cur_imu(1, 3) << std::endl;
        unsynced_imu_.pop_front();

        PubPos dr_temp;
        dr_temp.time = imu_front.time;
        dr_temp.x = T_prev_cur_imu(0, 3);
        dr_temp.y = T_prev_cur_imu(1, 3);
        dr_temp.z = T_prev_cur_imu(2, 3);
        Eigen::Quaterniond q = Eigen::Quaterniond(T_prev_cur_imu.block<3, 3>(0, 0));
        dr_temp.q = q;
        dr_buffer.push_back(dr_temp);
        pose_mtx_.lock();
        Twi = Twi * T_prev_cur_imu;
        pose_mtx_.unlock();
    }
}
Eigen::Matrix4d DrOdoFlow::GetPos() {
    Eigen::Matrix4d ret_pos;
    pose_mtx_.lock();
    ret_pos = Twi;
    pose_mtx_.unlock();
    return ret_pos;
}
Eigen::Matrix4d DrOdoFlow::Fusing_IMU_VelOdom(IMUData imu_front, IMUData imu_back) {
    Eigen::Matrix4d T_prev_cur_imu = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d J_prev_cur_imu = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_prev_cur_imu = Eigen::Matrix3d::Identity();

    R_prev_cur_imu = imu_front.orientation.getRoation().transpose() * imu_back.orientation.getRoation(); // Imu获取角度估计,计算变化值
    Eigen::AngleAxisd delta_se3(R_prev_cur_imu);
    double phi = delta_se3.angle();
    Eigen::Vector3d axis = delta_se3.axis();
    if (abs(phi) < 0.00001)
        J_prev_cur_imu = Eigen::Matrix3d::Identity();
    else
        J_prev_cur_imu = sin(phi) / phi * Eigen::Matrix3d::Identity() + (1 - sin(phi) / phi) * axis * axis.transpose() + ((1 - cos(phi)) / phi) * skew(axis); //视觉14讲，p73 4.26公式,右乘一个微小位移

    Eigen::Vector3d ang_vel(imu_back.angular_velocity.x, imu_back.angular_velocity.y, imu_back.angular_velocity.z);
    Eigen::Vector3d u = updateU(unsynced_velocity_, imu_front.time, imu_back.time,
                                ang_vel); //轮子速度在imu方向的投影
    //    Eigen::Vector3d u = updateU(unsynced_velocity_, imu_front.time, imu_back.time);
    T_prev_cur_imu.block(0, 0, 3, 3) << R_prev_cur_imu;
    T_prev_cur_imu.block(0, 3, 3, 1) << J_prev_cur_imu * u;
    T_prev_cur_imu.block(3, 0, 1, 4) << 0, 0, 0, 1;

    return T_prev_cur_imu;
}

void DrOdoFlow::WheelVel2IMUVel(VelocityData &vel_data, const Eigen::Vector3d &ang_vel) { // ang_vel measured by IMU, and is expressed in IMU coordinate
    // If all expressed in IMU coordinate, v_IMU = v_Wheel + ang_vel × r_Wheel_IMU
    Eigen::Matrix3d R_IMU_Wheel = IMU_T_WHEEL_.block<3, 3>(0, 0);
    Eigen::Vector3d r_Wheel_IMU = R_IMU_Wheel * IMU_T_WHEEL_.inverse().block<3, 1>(0, 3); // expressed in IMU coordinate ￥imu_to_wheel_
    Eigen::Vector3d v_Wheel(vel_data.linear_velocity.x, vel_data.linear_velocity.y,
                            vel_data.linear_velocity.z); // expressed in Wheel coordinate
    Eigen::Vector3d v_IMU = R_IMU_Wheel * v_Wheel + ang_vel.cross(r_Wheel_IMU);
    vel_data.linear_velocity.x = v_IMU(0);
    vel_data.linear_velocity.y = v_IMU(1);
    vel_data.linear_velocity.z = v_IMU(2);
}

Eigen::Vector3d DrOdoFlow::updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time) {
    std::deque<VelocityData> velocities;
    while (!unsynced_velocity.empty()) {
        VelocityData vel = unsynced_velocity.front();
        if (vel.time < front_time)
            unsynced_velocity.pop_front();
        else if (vel.time >= front_time && vel.time <= back_time) {
            unsynced_velocity.pop_front();
            velocities.push_back(vel);
        } else
            break;
    }
    VelocityData cur_velocity;
    int size = velocities.size();
    while (!velocities.empty()) {
        cur_velocity = cur_velocity + velocities.front();
        velocities.pop_front();
    }
    cur_velocity = size > 0 ? cur_velocity / size : cur_velocity;

    // 因为轮速记采样过慢，一段时间内认为保持匀速
    if (abs((front_time + back_time) / 2.0 - nonZeroLast.time) < 0.11 && size == 0) cur_velocity = nonZeroLast;

    VelocityData current_velocity_data_ = cur_velocity;

    current_velocity_data_.TransformCoordinate(IMU_T_WHEEL_);

    Eigen::Vector3d temp; //平移增量

    temp = (current_velocity_data_ * (back_time - front_time)).as_vector(); // 新车肯定用轮速计来算速度了

    return temp;
}

Eigen::Vector3d DrOdoFlow::updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time,
                                   const Eigen::Vector3d &ang_vel) {
    std::deque<VelocityData> velocities;
    while (!unsynced_velocity.empty()) {
        VelocityData vel = unsynced_velocity.front();
        if (vel.time < front_time)
            unsynced_velocity.pop_front();
        else if (vel.time >= front_time && vel.time <= back_time) {
            unsynced_velocity.pop_front();
            WheelVel2IMUVel(vel, ang_vel);
            velocities.push_back(vel);
        } else
            break;
    }
    VelocityData cur_velocity;
    int size = velocities.size();
    while (!velocities.empty()) {
        cur_velocity = cur_velocity + velocities.front();
        velocities.pop_front();
    }
    cur_velocity = size > 0 ? cur_velocity / size : cur_velocity;

    if (abs((front_time + back_time) / 2.0 - nonZeroLast.time) < 0.11 && size == 0) {
        cur_velocity = nonZeroLast;
        WheelVel2IMUVel(cur_velocity, ang_vel);
    }

    VelocityData current_velocity_data_ = cur_velocity;
    Eigen::Vector3d temp;

    temp = (current_velocity_data_ * (back_time - front_time)).as_vector();

    return temp;
}