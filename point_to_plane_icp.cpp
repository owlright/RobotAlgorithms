#include "common.h"
#include "icp3d.h"
#include <iostream>
#include <vector>

#include <random>

#include <pcl/kdtree/kdtree_flann.h>
DEFINE_string(source, "../EPFL/kneeling_lady_source.pcd", "第1个点云路径");
DEFINE_string(target, "../EPFL/kneeling_lady_target.pcd", "第2个点云路径");

using namespace Eigen;

// 生成测试点云数据
std::pair<MatrixXd, MatrixXd> generate_test_data(int n_points) {
    MatrixXd source(3, n_points);
    MatrixXd target(3, n_points);

    // 生成平面点云 (z=0)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    for (int i = 0; i < n_points; ++i) {
        source(0, i) = dis(gen);
        source(1, i) = dis(gen);
        source(2, i) = 0.0;
    }

    // 创建目标点云（添加旋转和平移）
    AngleAxisd rotation(M_PI/4, Vector3d(0, 0, 1)); // 45度绕Z轴旋转
    Vector3d translation(0.5, 0.2, 0.1);

    for (int i = 0; i < n_points; ++i) {
        Vector3d p = source.col(i);
        target.col(i) = rotation * p + translation;
    }

    return {source, target};
}

// 计算点云法向量（使用PCA）
MatrixXd compute_normals(const MatrixXd& points, int k=10) {
    int n = points.cols();
    MatrixXd normals(3, n);

    for (int i = 0; i < n; ++i) {
        // 找到最近邻点 (简化为随机点，实际应用中应使用KDTree)
        std::vector<int> indices;
        for (int j = 0; j < k; ++j) {
            indices.push_back((i + j) % n); // 简化版邻居选择
        }

        // 计算局部协方差矩阵
        Vector3d mean = Vector3d::Zero();
        for (int idx : indices) {
            mean += points.col(idx);
        }
        mean /= k;

        Matrix3d cov = Matrix3d::Zero();
        for (int idx : indices) {
            Vector3d v = points.col(idx) - mean;
            cov += v * v.transpose();
        }

        // PCA求法向量
        JacobiSVD<Matrix3d> svd(cov, ComputeFullV);
        normals.col(i) = svd.matrixV().col(2); // 最小特征值对应的特征向量
    }

    return normals;
}

// 点到平面ICP算法
Matrix4d point_to_plane_icp(const MatrixXd& source,
                           const MatrixXd& target,
                           const MatrixXd& target_normals,
                           int max_iterations = 20,
                           double tolerance = 1e-6) {
    int n = source.cols();
    Matrix4d T = Matrix4d::Identity();

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 初始化线性系统
        MatrixXd A = MatrixXd::Zero(6, 6);
        VectorXd b = VectorXd::Zero(6);

        double total_error = 0.0;
        int valid_points = 0;

        for (int i = 0; i < n; ++i) {
            // 变换当前点
            Vector4d p_homo;
            p_homo << source.col(i), 1.0;
            Vector4d p_trans_homo = T * p_homo;
            Vector3d p_trans = p_trans_homo.head<3>();

            // 简化：使用最近点作为对应点（实际应用中应使用KDTree）
            int j = i; // 这里简化对应关系

            Vector3d q = target.col(j);
            Vector3d n = target_normals.col(j);

            // 计算点到平面的距离
            double dist = n.dot(p_trans - q);
            total_error += std::abs(dist);

            // 构建雅可比矩阵
            Vector3d p_cross = p_trans.cross(n);
            VectorXd J(6);
            J << p_cross, n;

            // 累加线性系统
            A += J * J.transpose();
            b -= J * dist;

            valid_points++;
        }

        if (valid_points == 0) break;

        // 求解线性系统
        VectorXd x = A.ldlt().solve(b);

        // 提取旋转和平移
        Vector3d omega = x.head<3>();
        Vector3d t = x.tail<3>();

        // 创建增量变换
        Matrix3d R_inc = AngleAxisd(omega.norm(), omega.normalized()).toRotationMatrix();
        Matrix4d T_inc = Matrix4d::Identity();
        T_inc.block<3, 3>(0, 0) = R_inc;
        T_inc.block<3, 1>(0, 3) = t;

        // 更新变换
        T = T_inc * T;

        // 检查收敛
        double delta = x.norm();
        std::cout << "Iter " << iter << ": Error = " << total_error / valid_points
                  << ", Delta = " << delta << std::endl;

        if (delta < tolerance) {
            std::cout << "Converged at iteration " << iter << std::endl;
            break;
        }
    }

    return T;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    // 设置日志格式：包含时间戳、日志级别和文件位置
    FLAGS_log_prefix = true; // 启用日志前缀
    FLAGS_logtostderr = true; // 日志输出到标准错误
    google::ParseCommandLineFlags(&argc, &argv, true);


    ra::CloudPtr source(new ra::PointCloudType), target(new ra::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_source, *source);
    pcl::io::loadPCDFile(FLAGS_target, *target);
    ra::ICP3d icp;
    icp.SetSource(source);
    icp.SetTarget(target);
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
