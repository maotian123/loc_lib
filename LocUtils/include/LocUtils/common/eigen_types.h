#ifndef LocUtils_EIGEN_TYPES_H
#define LocUtils_EIGEN_TYPES_H

// 引入Eigen头文件与常用类型
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Matrix<char, 3, 1>;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4d = Eigen::Vector4d;
using Vec4f = Eigen::Vector4f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 15>;
using Vec18d = Eigen::Matrix<double, 18, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat32d = Eigen::Matrix<double, 3, 2>;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat96d = Eigen::Matrix<double, 9, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Mat18d = Eigen::Matrix<double, 18, 18>;

using VecXd = Eigen::Matrix<double, -1, 1>;
using MatXd = Eigen::Matrix<double, -1, -1>;
using MatX18d = Eigen::Matrix<double, -1, 18>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

const Mat3d Eye3d = Mat3d::Identity();
const Mat3f Eye3f = Mat3f::Identity();
const Mat4d Eye4d = Mat4d::Identity();
const Mat4f Eye4f = Mat4f::Identity();
const Vec3d Zero3d(0, 0, 0);
const Vec3f Zero3f(0, 0, 0);

// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SO2f = Sophus::SO2f;
using SE3 = Sophus::SE3d; 
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;

// using IdType = unsigned long;

// // Vec2i 可用于索引，定义它的小于号，用于构建以它为key的maps
namespace LocUtils {

/// 矢量比较
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

/// 矢量哈希
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

auto less_vec2i = [](const Vec2i& v1, const Vec2i& v2) {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
};

template <typename S>
inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4>& m) {
    /// 对R做归一化，防止sophus里的检查不过
    Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
    q.normalize();
    return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
}

template <typename S>
inline Eigen::Matrix<S, 4, 4> SE2TOMAT4(const Sophus::SE2<S>& se2) {
    Eigen::Matrix<S, 4, 4> mat = Eigen::Matrix<S, 4, 4>::Identity();
    
    mat.template block<2, 2>(0, 0) = se2.template rotationMatrix();
    mat.template block<2, 1>(0, 3) = se2.template translation();
    return mat;
}

template <typename S>
inline Eigen::Matrix<S, 4, 4> SE3TOMAT4(const Sophus::SE3<S>& se3) {
    Eigen::Matrix<S, 4, 4> mat = Eigen::Matrix<S, 4, 4>::Identity();
    
    mat.template block<3, 3>(0, 0) = se3.template rotationMatrix();
    mat.template block<3, 1>(0, 3) = se3.template translation();
    return mat;
}

template <typename S>
inline Vec3d RotationMatrixToEulerAngles(const Eigen::Matrix<S, 3, 3>& R)
{
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}

template <typename S>
inline Mat3d EulerAnglesToRotationMatrix(const Eigen::Matrix<S, 3, 1> &msg)
{
    Vec3d theta = msg.template cast<double>();
    Mat3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
            1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
            cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
            cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}


}  // namespace LocUtils

#endif  // LocUtils_EIGEN_TYPES_H
