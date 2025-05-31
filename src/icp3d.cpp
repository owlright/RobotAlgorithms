#include "icp3d.h"
namespace ra {
bool ICP3d::AlignP2P(SE3& init_pose)
{
    LOG(INFO) << "aligning point cloud using P2P ICP with initial pose: " << init_pose.translation().transpose();
    CHECK_NOTNULL(source_);
    CHECK_NOTNULL(target_);
    CHECK_NOTNULL(kdtree_);
    // 对点的索引，预先生成，为了并行使用
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::vector<bool> used_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());
    for (int iter = 0; iter < options_.max_iteration_; iter++) {
        for (int i = 0; i < index.size(); ++i) {
            auto q = ToVec3d(source_->points[i]);
            Vec3d q_trans = init_pose * q; // Apply the current pose to the source point
            std::vector<int> nn_indices;
            std::vector<float> nn_sqr_dists;
            kdtree_->nearestKSearch(ToPointType(q), 1, nn_indices, nn_sqr_dists);
            if (!nn_indices.empty()) {
                Vec3d p = ToVec3d(target_->points[nn_indices[0]]);
                double d = (p - q_trans).squaredNorm();
                if (d > options_.max_nn_distance_ * options_.max_nn_distance_) {
                    used_pts[i] = false;
                    continue; // 距离过大，跳过
                }
                used_pts[i] = true;

                // 计算误差
                Vec3d e = p - q_trans;
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = init_pose.so3().matrix() * SO3::hat(q);
                J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

                jacobians[i] = J;
                errors[i] = e;
            } else {
                used_pts[i] = false; // 没有找到最近邻点
            }
        }
    }
    return true;
}
bool ICP3d::AlignP2Line(SE3& init_pose) { return false; }
bool ICP3d::AlignP2Plane(SE3& init_pose) { return false; }
}
