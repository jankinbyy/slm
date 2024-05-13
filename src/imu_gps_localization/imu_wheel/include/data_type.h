#pragma once

#include "eigen3/Eigen/Core"
#include <deque>
#include "common.h"
constexpr double kDeg2Rad = M_PI / 180.;
constexpr double kRad2Deg = 180. / M_PI;
constexpr double kGravity = 9.8;
enum class Status { kValid,
                    kInvalid };
class IMUData {
public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct RPY {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    class Orientation {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

    public:
        void Normlize() {
            double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }

        RPY getRPY() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            Eigen::Vector3d ea = rx.eulerAngles(2, 1, 0);
            double roll, pitch, yaw;
            toEulerAngle(q, roll, pitch, yaw);
            RPY rpy;
            rpy.roll = roll;
            rpy.pitch = pitch;
            rpy.yaw = yaw;
            return rpy;
        }

        void Construct(Eigen::Quaterniond &quat) {
            x = quat.x();
            y = quat.y();
            z = quat.z();
            w = quat.w();
            Normlize();
        }

        Eigen::Matrix3d getRoation() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            return rx;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    //        geometry_msgs::Quaternion_ <std::allocator<void>> orientation;
    Orientation orientation;

public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();

    static bool SyncData(std::deque<IMUData> &UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time);

    template <typename T>
    static IMUData transform(const IMUData &imu_in, const Eigen::Matrix<T, 4, 4> &imu_to_other);
};
class VelocityData {
public:
    VelocityData() {
        linear_velocity.x = 0.0;
        linear_velocity.y = 0.0;
        linear_velocity.z = 0.0;
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = 0.0;
    }

    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

public:
    static bool SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                         double sync_time);

    void TransformCoordinate(Eigen::Matrix4d transform_matrix);

    VelocityData operator+(const VelocityData &v) {
        VelocityData vel;
        vel.time = this->time + v.time;
        vel.linear_velocity.x = this->linear_velocity.x + v.linear_velocity.x;
        vel.linear_velocity.y = this->linear_velocity.y + v.linear_velocity.y;
        vel.linear_velocity.z = this->linear_velocity.z + v.linear_velocity.z;
        vel.angular_velocity.x = this->angular_velocity.x + v.angular_velocity.x;
        vel.angular_velocity.y = this->angular_velocity.y + v.angular_velocity.y;
        vel.angular_velocity.z = this->angular_velocity.z + v.angular_velocity.z;
        return vel;
    }

    template <typename T>
    VelocityData operator/(const T &cnt) {
        VelocityData vel;
        vel.time = this->time / cnt;
        vel.linear_velocity.x = this->linear_velocity.x / cnt;
        vel.linear_velocity.y = this->linear_velocity.y / cnt;
        vel.linear_velocity.z = this->linear_velocity.z / cnt;
        vel.angular_velocity.x = this->angular_velocity.x / cnt;
        vel.angular_velocity.y = this->angular_velocity.y / cnt;
        vel.angular_velocity.z = this->angular_velocity.z / cnt;
        return vel;
    }

    template <typename T>
    VelocityData operator*(const T &cnt) {
        VelocityData vel;
        vel.time = this->time * cnt;
        vel.linear_velocity.x = this->linear_velocity.x * cnt;
        vel.linear_velocity.y = this->linear_velocity.y * cnt;
        vel.linear_velocity.z = this->linear_velocity.z * cnt;
        vel.angular_velocity.x = this->angular_velocity.x * cnt;
        vel.angular_velocity.y = this->angular_velocity.y * cnt;
        vel.angular_velocity.z = this->angular_velocity.z * cnt;
        return vel;
    }

    Eigen::Vector3d as_vector() {
        return Eigen::Vector3d(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    }
};
class PubPos {
public:
    double time;
    double x;
    double y;
    double z;
    Eigen::Quaterniond q;
};