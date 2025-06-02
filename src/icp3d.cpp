#include "icp3d.h"
#include <algorithm>    // std::for_each
#include <execution>    // std::execution::par_unseq
namespace ra {
bool ICP3d::AlignP2P_no_parallel(SE3& init_pose)
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

void parallel_for(size_t start, size_t end, const std::function<void(size_t)>& func) {
    size_t num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    size_t chunk_size = (end - start + num_threads - 1) / num_threads;
    for (size_t t = 0; t < num_threads; ++t) {
        size_t chunk_start = start + t * chunk_size;
        size_t chunk_end = std::min(chunk_start + chunk_size, end);

        threads.emplace_back([=]() {
            for (size_t i = chunk_start; i < chunk_end; ++i) {
                func(i);
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }
}

bool ICP3d::AlignP2P(SE3& init_pose)
{
    LOG(INFO) << "aligning point cloud using P2P ICP with initial pose: " << init_pose.translation().transpose();
    CHECK_NOTNULL(source_);
    CHECK_NOTNULL(target_);
    CHECK_NOTNULL(kdtree_);

    SE3 pose = init_pose;
    // 对点的索引，预先生成，为了并行使用
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());
    for (int iter = 0; iter < options_.max_iteration_; iter++) {
        parallel_for(0, index.size(), [&](size_t i) {
        // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int i) {
            auto q = ToVec3d(source_->points[i]);
            Vec3d q_trans = pose * q; // Apply the current pose to the source point
            std::vector<int> nn_indices;
            std::vector<float> nn_sqr_dists;
            kdtree_->nearestKSearch(ToPointType(q), 1, nn_indices, nn_sqr_dists);
            if (!nn_indices.empty()) {
                Vec3d p = ToVec3d(target_->points[nn_indices[0]]);
                double d = (p - q_trans).squaredNorm();
                if (d > options_.max_nn_distance_ * options_.max_nn_distance_) {
                    effect_pts[i] = false;
                    return; // 距离过大，跳过
                }
                effect_pts[i] = true;

                // 计算误差
                Vec3d e = p - q_trans;
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(q);
                J.block<3, 3>(0, 3) = -Mat3d::Identity();

                jacobians[i] = J;
                errors[i] = e;
            } else {
                effect_pts[i] = false; // 没有找到最近邻点
            }
        });
        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err
            = std::accumulate(index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
                [&jacobians, &errors, &effect_pts, &total_res, &effective_num](
                    const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                    if (!effect_pts[idx]) {
                        return pre;
                    } else {
                        total_res += errors[idx].dot(errors[idx]);
                        effective_num++;
                        return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
                            pre.second - jacobians[idx].transpose() * errors[idx]);
                    }
                });

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        // auto  dx2 = H.ldlt().solve(err);
        // std::cout << "dx: " << dx.transpose() << ", dx2: " << dx2.transpose() << std::endl;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

bool ICP3d::AlignP2Line(SE3& init_pose) { return false; }
bool ICP3d::AlignP2Plane(SE3& init_pose) { return false; }
}
