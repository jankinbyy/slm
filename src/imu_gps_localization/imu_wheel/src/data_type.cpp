#include "data_type.h"

Eigen::Matrix3f IMUData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}

bool IMUData::SyncData(std::deque<IMUData> &UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) {
            return false;
        }
        //            允许两帧需要同步的数据都在雷达之前
        if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size() > 2) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2) {
        return false;
    }

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 线性插值之后要归一化
    synced_data.orientation.Normlize();

    SyncedData.push_back(synced_data);

    return true;
}

template <typename T>
IMUData IMUData::transform(const IMUData &imu_in, const Eigen::Matrix<T, 4, 4> &imu_to_other) {
    IMUData imu_out = imu_in;
    Eigen::Matrix<T, 4, 4> extRot = imu_to_other.block(0, 0, 3, 3);
    // rotate acceleration
    Eigen::Matrix<T, 3, 1> acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y,
                               imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Matrix<T, 3, 1> gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaternion<T> q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                imu_in.orientation.z);
    Eigen::Quaternion<T> extQRPY(extRot);
    Eigen::Quaternion<T> q_final(extRot * q_from.toRotationMatrix() * extRot.transpose());

    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();
    return imu_out;
}
bool VelocityData::SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                            double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    bool diff_large = false;
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) return false;
        //            允许两帧需要同步的数据都在雷达之前
        if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size() > 2) {
            UnsyncedData.pop_front();
            continue;
        }
        //            fixme 频率
        if (sync_time - UnsyncedData.front().time > 0.5) {
            UnsyncedData.pop_front();
            diff_large = true;
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.5) {
            UnsyncedData.pop_front();
            diff_large = true;
            break;
        }
        break;
    }

    VelocityData synced_data;
    if (UnsyncedData.size() < 2) {
        //            因为轮速计代码设计，长时间不返数，默认轮速为0
        if (diff_large) {
            synced_data.time = sync_time;
            SyncedData.push_back(synced_data);
            return true;
        }
        return false;
    }

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    if (back_data.time - front_data.time == 0.0) {
        UnsyncedData.pop_front();
        return false;
    }

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    //        LOG_IF(INFO, Config::use_glog) << "front_scale: " << front_scale << "\tback_scale: " << back_scale;
    //        LOG_IF(INFO, Config::use_glog) << "SyncedData: " << SyncedData.size() << " back_data: " <<
    //        synced_data.linear_velocity.x << "\t" << synced_data.linear_velocity.y << "\t" <<
    //        synced_data.linear_velocity.z;
    SyncedData.push_back(synced_data);

    return true;
}

//    输入T_v^i
void VelocityData::TransformCoordinate(Eigen::Matrix4d transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();
    Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    // w_i = R_v^i * w_v
    // v_i = R_v^i * w_v x t_v^i + R_v^i * v_v
    w = t_R * w;
    v = t_R * v;
    Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * r(2) - w(2) * r(1);
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(0) * r(1) - w(1) * r(0);
    v = v - delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}