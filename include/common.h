#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace ra {

// 点云
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

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

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

}