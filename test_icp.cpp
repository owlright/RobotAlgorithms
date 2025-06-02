#include "common.h"
#include "icp3d.h"
#include <iostream>
#include <vector>

#include <random>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

DEFINE_string(source, "../EPFL/kneeling_lady_source.pcd", "第1个点云路径");
DEFINE_string(target, "../EPFL/kneeling_lady_target.pcd", "第2个点云路径");

using namespace Eigen;

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    // 设置日志格式：包含时间戳、日志级别和文件位置
    FLAGS_log_prefix = true; // 启用日志前缀
    FLAGS_logtostderr = true; // 日志输出到标准错误
    google::SetUsageMessage("ICP3d test program.\n"
                            "Usage: ./test_icp --source <source.pcd> --target <target.pcd>");
    google::ParseCommandLineFlags(&argc, &argv, true);

    ra::CloudPtr source(new ra::PointCloudType), target(new ra::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_source, *source);
    pcl::io::loadPCDFile(FLAGS_target, *target);
    ra::ICP3d icp;
    icp.SetSource(source);
    icp.SetTarget(target);
    icp.BuildKdTree();

    bool success = false;
    // ra::benchmark([&]() {success = icp.AlignP2P_no_parallel(init_pose);}, "ICP P2P Parallel Version", 1);
    ra::benchmark(
        [&]() {
            ra::SE3 pose;
            success = icp.AlignP2P(pose);
            ra::CloudPtr source_trans(new ra::PointCloudType);
            pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
            ra::SaveCloudToFile("./icp_trans.pcd", *source_trans);
            if (success) {
                LOG(INFO) << "ICP P2P alignment succeeded with final pose: " << pose.translation().transpose();
            } else {
                LOG(ERROR) << "ICP P2P alignment failed.";
            }
        },
        "ICP P2P", 10);

    // // 生成测试数据
    // int n_points = 100;
    // auto [source, target] = generate_test_data(n_points);

    // // 计算目标点云法向量
    // MatrixXd target_normals = compute_normals(target);

    // // 运行ICP算法
    // Matrix4d transformation = point_to_plane_icp(source, target, target_normals);

    // // 打印结果
    // LOG(INFO) << "Estimated transformation matrix:\n" << transformation;

    // // 计算真实变换（用于验证）
    // Matrix4d true_transform = Matrix4d::Identity();
    // true_transform.block<3, 3>(0, 0) = AngleAxisd(M_PI/4, Vector3d(0, 0, 1)).toRotationMatrix();
    // true_transform(0, 3) = 0.5;
    // true_transform(1, 3) = 0.2;
    // true_transform(2, 3) = 0.1;

    // std::cout << "\nTrue transformation matrix:\n" << true_transform << std::endl;
    // std::cout << "\nError:\n" << transformation - true_transform << std::endl;

    return 0;
}
