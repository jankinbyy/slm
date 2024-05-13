#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"
template <typename T>
void toEulerAngle(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw) {
    // roll (x-axis rotation)
    T sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    T cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}
inline Eigen::Matrix3d SkewMat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
    return w;
}
template <typename T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 3, 3> skew_matrix = Eigen::Matrix<T, 3, 3>::Identity();

    skew_matrix << 0, -axis(2, 0), axis(1, 0), axis(2, 0), 0, -axis(0, 0), -axis(1, 0), axis(0, 0), 0;

    return skew_matrix;
}
template <typename T>
inline Eigen::Matrix<T, 4, 4> EigenIsoInv(const Eigen::Matrix<T, 4, 4> &Tcw) {
    Eigen::Matrix<T, 3, 3> Rcw = Tcw.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> tcw = Tcw.block(0, 3, 3, 1);
    Eigen::Matrix<T, 3, 3> Rwc = Rcw.transpose();
    Eigen::Matrix<T, 3, 1> twc = -Rwc * tcw;

    Eigen::Matrix<T, 4, 4> Twc = Eigen::Matrix<T, 4, 4>::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = twc;

    return Twc;
}