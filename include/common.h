#pragma once

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h> // for KdTree
#include <pcl/point_types.h>

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

// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt)
{
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}

/**
 * 统计代码运行时间
 * @tparam FuncT
 * @param func  被调用函数
 * @param func_name 函数名
 * @param times 调用次数
 */
template <typename FuncT>
void benchmark(FuncT&& func, const std::string& func_name = "", int times = 10)
{
    double total_time = 0;
    for (int i = 0; i < times; ++i) {
        auto t1 = std::chrono::high_resolution_clock::now();
        func();
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time += std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    }

    LOG(INFO) << "方法 " << func_name << " 平均调用时间/次数: " << total_time / times << "/" << times << " 毫秒.";
}
template <typename CloudType>
void SaveCloudToFile(const std::string& filePath, CloudType& cloud)
{
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}
}