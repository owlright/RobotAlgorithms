#include "ndt3d.h"
#include "util.h"
namespace ra {
void NDT3d::BuildVoxels()
{
    CHECK_NOTNULL(target_);
    CHECK(!target_->empty()) << "Target point cloud is empty!";
    std::vector<size_t> index(target_->size());
    std::for_each(index.begin(), index.end(), [n = 0](size_t& i) mutable { i = n++; });
    std::for_each(index.begin(), index.end(), [this](const size_t& i) {
        Vec3d pt = ToVec3d(target_->points[i]);
        auto key = (pt * options_.inv_voxel_size_).cast<int>();
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({ key, { i } });
        } else {
            grids_[key].idx_.emplace_back(i);
        }
    });
    // 计算每个体素中的均值和协方差
    util::parallel_for(grids_.begin(), grids_.end(), [this](auto& v) {
        if (v.second.idx_.size() > options_.min_pts_in_voxel_) {
            // 要求至少有３个点
            math::ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_, [this](const size_t& idx) {
                return ToVec3d(target_->points[idx]);
            });
            // SVD 检查最大与最小奇异值，限制最小奇异值

            Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }

            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

            // v.second.info_ = (v.second.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免出nan
            v.second.inv_sigma_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });
    // 删除点数不够的
    for (auto iter = grids_.begin(); iter != grids_.end();) {
        if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
            iter++;
        } else {
            iter = grids_.erase(iter);
        }
    }
}
}
