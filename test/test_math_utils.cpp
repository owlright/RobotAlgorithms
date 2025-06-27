#include "common/math_utils.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

// 全局初始化
class TestEnvironment : public ::testing::Environment {
public:
    void SetUp() override
    {
        google::InitGoogleLogging("test_math_utils");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;
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