#include "common/math_utils.h"
#include <Eigen/Dense>
#include <fstream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
using namespace ra;
DEFINE_int32(num_tested_points_plane, 1000, "Number of points to test plane fitting");

template <typename S>
class Plane3DEquation {
public:
    using V3 = Eigen::Matrix<S, 3, 1>;
    using V4 = Eigen::Matrix<S, 4, 1>;
    Plane3DEquation()
        : coeffs_(V4::Zero())
    {
    }

    /* -------------------------------- 构造平面的不同方法 ------------------------------- */
    // 从平面系数构造
    explicit Plane3DEquation(const V4& coeffs)
        : coeffs_(coeffs)
    {
        normalize();
    }

    // 从法向量和点构造
    Plane3DEquation(const V3& normal, const V3& point)
    {
        V3 n = normal.normalized();
        S d = -n.dot(point);
        coeffs_ << n, d;
    }

    // 从三个点构造平面
    Plane3DEquation(const V3& p1, const V3& p2, const V3& p3)
    {
        V3 v1 = p2 - p1;
        V3 v2 = p3 - p1;
        V3 normal = v1.cross(v2).normalized();
        S d = -normal.dot(p1);
        coeffs_ << normal, d;
    }
    /* -------------------------------------------------------------------------- */

    // 使用 FitPlane 函数拟合平面
    // clang-format off
    bool fitFromPoints(const std::vector<V3>& points, double eps = 0.1) {
        bool success = ra::FitPlane(points, coeffs_, eps, ra::PlaneFittingMethod::SVD);
        if (success) {
            normalize();
        } else {
            LOG(WARNING) << "Plane fitting failed";
        }
        return success;
    }
    // clang-format on

    // 归一化平面系数
    void normalize()
    {
        S norm = coeffs_.template head<3>().norm();
        if (norm > 1e-10) {
            coeffs_ /= norm;
        }
    }

    void setCoeffs(const V4& coeffs)
    {
        coeffs_ = coeffs;
        normalize();
    }

    /* --------------------------------- 获取数据函数 --------------------------------- */
    const V4& coeffs() const { return coeffs_; }
    V3 normal() const { return coeffs_.template head<3>(); }

    /* ---------------------------------- 生成数据 ---------------------------------- */
    void setSeed(unsigned int seed) { gen_.seed(seed); }
    std::vector<V3> generateRandomPoints(int num_points, S range = 10.0, S noise_std = 0.01)
    {
        std::vector<V3> points;
        points.reserve(num_points);
        std::normal_distribution<S> noise(0.0, noise_std);
        std::uniform_real_distribution<S> dis(-range, range);
        // Generate random points on the plane
        for (int i = 0; i < num_points; ++i) {
            // Generate two random parameters for plane parametrization
            S u = dis(gen_);
            S v = dis(gen_);

            // Create two orthogonal vectors on the plane
            auto normal = this->normal();
            V3 tangent1, tangent2;

            // Find a vector not parallel to normal
            if (std::abs(normal(0)) < 0.9) {
                tangent1 = V3(1, 0, 0);
            } else {
                tangent1 = V3(0, 1, 0);
            }

            // 使用 Gram-Schmidt 正交化投影到平面
            tangent1 = tangent1 - tangent1.dot(normal) * normal;
            tangent1.normalize();
            tangent2 = normal.cross(tangent1); // 叉积得到垂直于normal和tangen1的向量

            // Point on plane: base_point + u*tangent1 + v*tangent2
            // ! 注意coeffs一定要是归一化的
            V3 base_point = -coeffs_(3) * normal; // 原点到平面的最短距离
            V3 point = base_point + u * tangent1 + v * tangent2;

            // Add small noise
            point += V3(noise(gen_), noise(gen_), noise(gen_));

            points.push_back(point);
        }
        return points;
    }

private:
    V4 coeffs_;        // 平面系数 [a, b, c, d] 对应 ax + by + cz + d = 0
    std::mt19937 gen_; // 随机数生成器
};

// 初始化测试环境
// // 自定义日志前缀函数 需要glog0.7.0以上版本
// void CustomLogPrefix(std::ostream& s, const google::LogMessage& l, void*)
// {
//     // 获取当前时间
//     auto now = std::chrono::system_clock::now();
//     auto time_t = std::chrono::system_clock::to_time_t(now);
//     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

//     // 获取日志等级
//     const char* level = "";
//     switch (l.severity()) {
//     case google::GLOG_INFO:
//         level = "INFO";
//         break;
//     case google::GLOG_WARNING:
//         level = "WARNING";
//         break;
//     case google::GLOG_ERROR:
//         level = "ERROR";
//         break;
//     case google::GLOG_FATAL:
//         level = "FATAL";
//         break;
//     default:
//         level = "UNKNOWN";
//         break;
//     }
//     s << "[" << level << "]"
//       << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
//       << ms.count() << "]"
//       << "[" << l.basename() << ":" << l.line() << "] ";
// }

class TestEnvironment : public ::testing::Environment {
public:
    void SetUp() override
    {
        google::InitGoogleLogging("test_math_utils");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;
        // 安装自定义日志前缀 需要glog0.7.0以上版本
        // google::InstallPrefixFormatter(&CustomLogPrefix, nullptr);
    }

    void TearDown() override { google::ShutdownGoogleLogging(); }
};

// 注册全局环境
GTEST_API_ int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_color = "yes"; // 设置彩色输出
    ::testing::FLAGS_gtest_brief = true;  // 简洁输出
    ::testing::AddGlobalTestEnvironment(new TestEnvironment);
    return RUN_ALL_TESTS();
}

TEST(LessVecTest, CompareVectors3)
{
    ra::less_vec<3> comparator;

    // Case 1: v1 < v2
    Eigen::Matrix<int, 3, 1> v1(1, 2, 3);
    Eigen::Matrix<int, 3, 1> v2(2, 2, 3);
    EXPECT_TRUE(comparator(v1, v2)); // v1 < v2

    // Case 2: v1 > v2
    v1 = Eigen::Matrix<int, 3, 1>(2, 2, 3);
    v2 = Eigen::Matrix<int, 3, 1>(1, 2, 3);
    EXPECT_FALSE(comparator(v1, v2)); // v1 > v2

    // Case 3: v1 == v2
    v1 = Eigen::Matrix<int, 3, 1>(1, 2, 3);
    v2 = Eigen::Matrix<int, 3, 1>(1, 2, 3);
    EXPECT_FALSE(comparator(v1, v2)); // v1 == v2

    // Case 4: Compare second element
    v1 = Eigen::Matrix<int, 3, 1>(1, 1, 3);
    v2 = Eigen::Matrix<int, 3, 1>(1, 2, 3);
    EXPECT_TRUE(comparator(v1, v2)); // v1 < v2 (second element)

    // Case 5: Compare third element
    v1 = Eigen::Matrix<int, 3, 1>(1, 2, 2);
    v2 = Eigen::Matrix<int, 3, 1>(1, 2, 3);
    EXPECT_TRUE(comparator(v1, v2)); // v1 < v2 (third element)
}

TEST(LessVecTest, CompareVectors2)
{
    ra::less_vec<2> comparator;

    // Case 1: v1 < v2
    Eigen::Matrix<int, 2, 1> v1(1, 2);
    Eigen::Matrix<int, 2, 1> v2(2, 2);
    EXPECT_TRUE(comparator(v1, v2)); // v1 < v2

    // Case 2: v1 > v2
    v1 = Eigen::Matrix<int, 2, 1>(2, 2);
    v2 = Eigen::Matrix<int, 2, 1>(1, 2);
    EXPECT_FALSE(comparator(v1, v2)); // v1 > v2

    // Case 3: v1 == v2
    v1 = Eigen::Matrix<int, 2, 1>(1, 2);
    v2 = Eigen::Matrix<int, 2, 1>(1, 2);
    EXPECT_FALSE(comparator(v1, v2)); // v1 == v2

    // Case 4: Compare second element
    v1 = Eigen::Matrix<int, 2, 1>(1, 1);
    v2 = Eigen::Matrix<int, 2, 1>(1, 2);
    EXPECT_TRUE(comparator(v1, v2)); // v1 < v2 (second element)
}

TEST(FitPlaneTest, BasicFunctionality)
{
    std::vector<Eigen::Vector3d> points = {
        { 1.0, 2.0, 3.0 },
        { 4.0, 5.0, 6.0 },
        { 7.0, 8.0, 9.0 },
        { 2.0, 3.0, 1.0 },
        { 3.0, 4.0, 2.0 },
        { 5.0, 6.0, 4.0 },
        { 6.0, 7.0, 5.0 },
        { 8.0, 9.0, 7.0 }
    };
    Eigen::Vector4d plane_coeffs;
    EXPECT_TRUE(ra::FitPlane(points, plane_coeffs));
    LOG(INFO) << "Plane coefficients: " << plane_coeffs.transpose();
    // Check if the plane coefficients are correct
    EXPECT_NEAR(plane_coeffs[0], 0.573, 5e-3);
    EXPECT_NEAR(plane_coeffs[1], -0.573, 5e-3);
    EXPECT_NEAR(plane_coeffs[2], 0.0, 5e-3);
    EXPECT_NEAR(plane_coeffs[3], 0.573, 5e-3); // Plane equation: x + y + z - 6 = 0
}

TEST(FitPlaneTest, RandomPoints)
{
    // a*x + b*y + c*z + d = 0
    // [a, b, c] * [x, y, z]^T + d = 0
    // n * P + d = 0

    ra::Vec4d true_plane_coeffs(1, 2, 3, 4);
    Plane3DEquation<double> plane(true_plane_coeffs);
    LOG(INFO) << "True plane coefficients:" << plane.coeffs().transpose();
    plane.setSeed(42);
    auto points = plane.generateRandomPoints(FLAGS_num_tested_points_plane, 10.0, 0.01);
    std::ofstream datafile("../scripts/plane_points.txt");
    if (datafile.is_open()) {
        for (const auto& point : points) {
            datafile << point(0) << " " << point(1) << " " << point(2) << std::endl;
        }
        datafile.close();
        LOG(INFO) << "Successfully wrote " << points.size() << " points to plane_points.txt";
    }
    // Fit plane to generated points
    Plane3DEquation<double> fitted_plane;
    EXPECT_TRUE(fitted_plane.fitFromPoints(points));
    LOG(INFO) << "Fitted plane coefficients: " << fitted_plane.coeffs().transpose();
    points = fitted_plane.generateRandomPoints(FLAGS_num_tested_points_plane, 10.0, 0.01);
    std::ofstream fitted_datafile("../scripts/fitted_plane_points.txt");
    if (fitted_datafile.is_open()) {
        for (const auto& point : points) {
            fitted_datafile << point(0) << " " << point(1) << " " << point(2) << std::endl;
        }
        fitted_datafile.close();
        LOG(INFO) << "Successfully wrote " << points.size() << " fitted points to fitted_plane_points.txt";
    }
    // Check if fitted coefficients are close to true coefficients
    // Note: plane normal can point in opposite direction
    double dot_product = fitted_plane.normal().dot(plane.normal());
    if (dot_product < 0) {
        fitted_plane.setCoeffs(-fitted_plane.coeffs());
        LOG(INFO) << "Fitted plane coefficients flipped to match true coefficients direction";
    }

    EXPECT_NEAR(fitted_plane.coeffs()(0), plane.coeffs()(0), 1e-3);
    EXPECT_NEAR(fitted_plane.coeffs()(1), plane.coeffs()(1), 1e-3);
    EXPECT_NEAR(fitted_plane.coeffs()(2), plane.coeffs()(2), 1e-3);
    EXPECT_NEAR(fitted_plane.coeffs()(3), plane.coeffs()(3), 1e-3);
}