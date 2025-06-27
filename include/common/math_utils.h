#pragma once
#include "common.h"

namespace ra {
using Eigen::Matrix;
using Eigen::MatrixXd;
using std::vector;

template <typename S>
bool FitPlane(vector<Matrix<S, 3, 1>>& data, Matrix<S, 4, 1>& plane_coeffs, double eps = 1e-2)
{
    if (data.size() < 4) {
        LOG(WARNING) << "Not enough points to fit a plane";
        return false;
    }
    MatrixXd A(data.size(), 4);
    for (size_t i = 0; i < data.size(); ++i) {
        A.row(i) << data[i][0], data[i][1], data[i][2], 1.0;
    }
    // JacobiSVD 类的一个枚举选项，用于指定在计算奇异值分解（SVD）时是否需要计算右奇异向量矩阵
    // V，以及计算的形式（完整或精简）。
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    auto V = svd.matrixV(); // 获取右奇异向量矩阵
    if (V.cols() < 4) {
        LOG(WARNING) << "Not enough columns in V matrix for plane fitting";
        return false; // 如果V矩阵的列数小于4，说明数据不适合拟合平面
    }
    plane_coeffs = V.col(3); // 取最后一列作为平面系数
    for (size_t i = 0; i < data.size(); ++i) {
        // 假设平面方程为$n \cdot x + d = 0$
        // 其中n是平面的法向量，x是平面上的点，等式成立
        // 当点$x_k$不是平面上的点时，$n \cdot x_k + d$ 是点到平面的距离。
        double err = plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3];
        // 如果有某个点到平面的距离大于eps，则认为拟合失败
        if (err * err > eps) {
            LOG(INFO) << "Point " << i << " is too far from the fitted plane: " << err;
            return false; // 如果有点距离平面太远，则返回false
        }
    }
    return true;
}

/// less of vector
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const
{
    if (v1[0] != v2[0]) {
        return v1[0] < v2[0];
    }
    return v1[1] < v2[1];
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const
{
    if (v1[0] != v2[0]) {
        return v1[0] < v2[0];
    }
    if (v1[1] != v2[1]) {
        return v1[1] < v2[1];
    }
    return v1[2] < v2[2];
}

} // namespace ra