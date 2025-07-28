#pragma once
#include "common.h"

namespace ra {
using Eigen::Matrix;
using Eigen::MatrixXd;
using std::vector;

// 平面拟合算法类型
enum class PlaneFittingMethod {
    SVD,                 // 奇异值分解 (默认)
    EIGEN_DECOMPOSITION, // 特征值分解
    LEAST_SQUARES,       // 最小二乘法
    RANSAC               // RANSAC算法 (未来扩展)
};

template <typename S>
bool FitPlane(
    const vector<Matrix<S, 3, 1>>& data, Matrix<S, 4, 1>& plane_coeffs, double eps = 0.1,
    PlaneFittingMethod method = PlaneFittingMethod::SVD)
{
    if (data.size() < 4) {
        LOG(WARNING) << "Not enough points to fit a plane";
        return false;
    }
    MatrixXd A(data.size(), 4);
    for (size_t i = 0; i < data.size(); ++i) {
        A.row(i) << data[i][0], data[i][1], data[i][2], 1.0;
    }
    switch (method) {
    case PlaneFittingMethod::SVD: {
        // 使用奇异值分解（SVD）来拟合平面
        // Eigen::ComputeThinV 是 JacobiSVD 类的一个枚举选项，用于指定在计算奇异值分解（SVD）时
        // 是否需要计算右奇异向量矩阵V，以及计算的形式（完整或精简）。
        Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
        auto V = svd.matrixV(); // 获取右奇异向量矩阵
        if (V.cols() < 4) {
            LOG(WARNING) << "Not enough columns in V matrix for plane fitting";
            return false; // 如果V矩阵的列数小于4，说明数据不适合拟合平面
        }
        plane_coeffs = V.col(3); // 取最后一列作为平面系数

        break;
    }
    case PlaneFittingMethod::EIGEN_DECOMPOSITION: {
        // 使用特征值分解来拟合平面
        // 构建ATA矩阵
        Eigen::Matrix<S, 4, 4> ATA = A.transpose() * A;
        // 对ATA矩阵进行特征值分解
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<S, 4, 4>> eigen_solver(ATA);
        if (eigen_solver.info() != Eigen::Success) {
            LOG(WARNING) << "Eigenvalue decomposition failed";
            return false;
        }
        // 最小特征值对应的特征向量即为平面系数
        // 特征值是按升序排列的，所以第0列对应最小特征值
        plane_coeffs = eigen_solver.eigenvectors().col(0).template cast<S>();
        break;
    }
    default: {
        LOG(WARNING) << "Unsupported plane fitting method";
        return false; // 如果方法不支持，返回false
    }
    }
    // 归一化平面系数（使法向量为单位向量）
    S norm = plane_coeffs.template head<3>().norm();
    if (norm < 1e-10) {
        LOG(WARNING) << "Degenerate plane normal vector";
        return false;
    }
    // 验证拟合质量
    int outliers = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        // 假设平面方程为$n \cdot x + d = 0$
        // 其中n是平面的法向量，x是平面上的点，等式成立
        // 当点$x_k$不是平面上的点时，$n \cdot x_k + d$ 是点到平面的距离。
        double err = (plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3]) / norm;
        // 如果有某个点到平面的距离大于eps，则认为拟合失败
        if (std::abs(err) > eps) {
            outliers++;
            LOG(INFO) << "Point " << i << " is too far from the fitted plane: " << err;
        }
    }
    // 如果外点太多，认为拟合失败
    if (outliers > data.size() * 0.1) { // 允许10%的外点
        LOG(WARNING) << "Too many outliers: " << outliers << "/" << data.size();
        return false;
    }
    LOG(INFO) << "Plane fitting successful with " << outliers << "/" << data.size() << " outliers";
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