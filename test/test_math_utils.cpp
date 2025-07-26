#include "common/math_utils.h"
#include <Eigen/Dense>
#include <fstream>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
using namespace ra;
DEFINE_int32(num_tested_points_plane, 1000, "Number of points to test plane fitting");
// 初始化测试环境
// 自定义日志前缀函数
void CustomLogPrefix(std::ostream& s, const google::LogMessage& l, void*)
{
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // 获取日志等级
    const char* level = "";
    switch (l.severity()) {
    case google::GLOG_INFO:
        level = "INFO";
        break;
    case google::GLOG_WARNING:
        level = "WARNING";
        break;
    case google::GLOG_ERROR:
        level = "ERROR";
        break;
    case google::GLOG_FATAL:
        level = "FATAL";
        break;
    default:
        level = "UNKNOWN";
        break;
    }
    s << "[" << level << "]"
      << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
      << ms.count() << "]"
      << "[" << l.basename() << ":" << l.line() << "] ";
}

class TestEnvironment : public ::testing::Environment {
public:
    void SetUp() override
    {
        google::InitGoogleLogging("test_math_utils");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;
        // 安装自定义日志前缀
        google::InstallPrefixFormatter(&CustomLogPrefix, nullptr);
    }

    void TearDown() override { google::ShutdownGoogleLogging(); }
};

// 注册全局环境
GTEST_API_ int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_color = "yes"; // 设置彩色输出
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

    // Check if the plane coefficients are correct
    EXPECT_NEAR(plane_coeffs[0], 0.573, 5e-3);
    EXPECT_NEAR(plane_coeffs[1], -0.573, 5e-3);
    EXPECT_NEAR(plane_coeffs[2], 0.0, 5e-3);
    EXPECT_NEAR(plane_coeffs[3], 0.573, 5e-3); // Plane equation: x + y + z - 6 = 0
}