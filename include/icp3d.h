#pragma once
#include "common.h"
#include <numeric>  // for std::accumulate
namespace ra {

class ICP3d {
public:
    ICP3d() : source_(nullptr), target_(nullptr), source_center_(Vec3d::Zero()), target_center_(Vec3d::Zero()) {}
    ~ICP3d() = default;
    void SetSource(CloudPtr source)
    {
        source_ = source;
        source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                             [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) / source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    void SetTarget(CloudPtr target)
    {
        target_ = target;
        target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                             [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) / target_->size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }
    /// 使用gauss-newton方法进行配准, 点到点
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
};

}