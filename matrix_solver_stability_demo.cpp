#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

int main()
{
    // 设置随机数种子
    // srand(42);
    Vector3d x(1, 2, 3);
    // MatrixXd A = MatrixXd::Random(40, 3) ; // 随机生成一个 3x3 矩阵
    Matrix3d A;
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9; // 手动设置一个矩阵
    cout << "A:" << endl;
    cout << A << endl;
    cout << "x:" << endl;
    cout << x << endl;  // 输出 A * x
    Vector3d b = A * x; // 计算 Ax
    cout << "b=Ax:" << endl;
    cout << b << endl;            // 输出 b
    auto AtA = A.transpose() * A; // 确保矩阵是对称的
    cout << "AtA:" << endl;
    cout << AtA << endl;
    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(AtA);
    cout << (solver.info() == Eigen::Success) << endl; // 检查求解器状态
    // 输出特征值
    cout << "特征值:" << endl;
    auto eigenvalues = solver.eigenvalues();
    cout << eigenvalues << endl;
    // 输出特征向量
    cout << "matV:" << endl;
    auto matV = solver.eigenvectors();
    cout << matV << endl;
    auto matV2 = matV;
    double cond = eigenvalues.maxCoeff() / eigenvalues.minCoeff();
    if (cond == std::numeric_limits<double>::infinity()) {
        cout << "A cond: 无穷大" << endl;
    } else {
        cout << "A cond: " << cond << endl;
    }

    float eignThre[3] = { 1, 2, 10 };
    int count = 0;
    for (int i = 0; i < 3; ++i) {
        if (eigenvalues[i] < eignThre[i]) {
            matV2.col(i) = Vector3d::Zero();
            count++;
        } else {
            break;
        }
    }
    cout << "matV2:" << endl;
    cout << matV2 << endl;
    auto matP = matV.transpose() * matV2;
    cout << "投影矩阵:\n" << matP << endl;
    cout << matP * x << endl; // 验证投影矩阵的效果
    // for (int i = 0; i < 3; ++i) {
    //     // 提取第 i 个特征值和特征向量
    //     auto eigenvalue = eigenvalues[i];
    //     auto eigenvector = matV.col(i);

    //     // 计算 A * eigenvector 和 eigenvalue * eigenvector
    //     auto lhs = A * eigenvector; // 左侧: A * 特征向量
    //     auto rhs = eigenvalue * eigenvector; // 右侧: 特征值 * 特征向量

    //     // 输出验证结果
    //     cout << "特征值 " << i + 1 << ": " << eigenvalue << endl;
    //     cout << "A * v: " << lhs.transpose() << endl;
    //     cout << "λ * v: " << rhs.transpose() << endl;
    //     cout << "误差: " << (lhs - rhs).norm() << endl;
    //     cout << "------------------------" << endl;
    // }
    return 0;
}
