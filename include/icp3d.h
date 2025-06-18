#pragma once
#include "common.h"
#include <numeric> // for std::accumulate
#ifdef __APPLE__
// macOS 平台使用 std::make_shared
#define MAKE_SHARED std::make_shared
#else
// 其他平台使用 boost::make_shared
#define MAKE_SHARED boost::make_shared
#endif

namespace ra {

class ICP3d {
public:
    struct Options {
        int max_iteration_ = 20;               // 最大迭代次数
        double max_nn_distance_ = 1.0;         // 点到点最近邻查找时阈值
        double max_plane_distance_ = 0.05;     // 平面最近邻查找时阈值
        double max_line_distance_ = 0.5;       // 点线最近邻查找时阈值
        int min_effective_pts_ = 10;           // 最近邻点数阈值
        double eps_ = 1e-2;                    // 收敛判定条件
        bool use_initial_translation_ = false; // 是否使用初始位姿中的平移估计
    };

public:
    ICP3d()
        : source_(nullptr)
        , target_(nullptr)
        , source_center_(Vec3d::Zero())
        , target_center_(Vec3d::Zero())
    {
    }
    ~ICP3d() = default;
    void SetSource(CloudPtr source)
    {
        source_ = source;
        CHECK_NOTNULL(source_);
        source_center_ = std::accumulate(
                             source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                             [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); })
            / source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    void SetTarget(CloudPtr target)
    {
        target_ = target;
        CHECK_NOTNULL(target_);
        target_center_ = std::accumulate(
                             target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                             [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); })
            / target_->size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }

    void BuildKdTree()
    {
        CHECK_NOTNULL(target_);
        kdtree_ = MAKE_SHARED<pcl::KdTreeFLANN<PointType>>();
        kdtree_->setInputCloud(target_);
        LOG(INFO) << "KdTree built for target point cloud.";
    }
    /// 使用gauss-newton方法进行配准, 点到点
    bool AlignP2P_no_parallel(SE3& init_pose);

    bool AlignP2P(SE3& init_pose);

    /// 基于gauss-newton的点线ICP
    bool AlignP2Line(SE3& init_pose);

    /// 基于gauss-newton的点面ICP
    bool AlignP2Plane(SE3& init_pose);

private:
    CloudPtr source_;
    CloudPtr target_;
    Vec3d source_center_;
    Vec3d target_center_;

    Options options_;
    // KdTree
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_;
    bool gt_set_ = false;
    SE3 gt_pose_; // ground truth pose, if available
};

}