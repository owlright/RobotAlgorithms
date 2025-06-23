#include "imu_integration.h"
namespace ra {

void IMUIntegration::AddIMU(const IMU& imu)
{
    double dt = imu.timestamp_ - timestamp_;
    if (dt > 0 && dt < 0.1) {
        // 假设IMU时间间隔在0至0.1以内
        p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
        v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
        R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
    }

    // 更新时间
    timestamp_ = imu.timestamp_;
}

}