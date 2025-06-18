#pragma once
#include "common.h"

namespace ra {
using namespace Eigen;

template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const
    {
        return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
    }
};

class NDT3d {
public:
    enum class NearbyType {
        CENTER,
        NEARBY6,
    };

    struct Options {
        int max_iteration_ = 20;       // 最大迭代次数
        double voxel_size_ = 1.0;      // 体素大小
        double inv_voxel_size_ = 1.0;  //
        int min_effective_pts_ = 10;   // 最近邻点数阈值
        int min_pts_in_voxel_ = 3;     // 每个栅格中最小点数
        double eps_ = 1e-2;            // 收敛判定条件
        double res_outlier_th_ = 20.0; // 异常值拒绝阈值
        bool remove_centroid_ = false; // 是否计算两个点云中心并移除中心？

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    using KeyType = Matrix<int, 3, 1>;
    struct Voxel {
        Voxel() { }
        Voxel(size_t id) { idx_.emplace_back(id); }

        std::vector<size_t> idx_;         // 点云中心索引
        Vec3d mu_ = Vec3d::Zero();        // 均值
        Mat3d sigma_ = Mat3d::Zero();     // 协方差矩阵
        Mat3d inv_sigma_ = Mat3d::Zero(); // 协方差矩阵的逆
    };

private:
    void BuildVoxels();

    void GenerateNearbyGrids();

    CloudPtr source_ = nullptr;
    CloudPtr target_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();
    Options options_;
    std::unordered_map<KeyType, Voxel, hash_vec<3>> grids_;
    std::vector<KeyType> nearby_grids_;
};
}