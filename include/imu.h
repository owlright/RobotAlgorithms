//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_IMU_H
#define MAPPING_IMU_H

#include "common.h"
#include <memory>
namespace ra {

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce)
        : timestamp_(t)
        , gyro_(gyro)
        , acce_(acce)
    {
    }

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};

} // namespace sad

using IMUPtr = std::shared_ptr<ra::IMU>;

#endif // MAPPING_IMU_H