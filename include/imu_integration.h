#include "common.h"
#include "imu.h"
#include "nav_state.h"

namespace ra {

class IMUIntegration {
public:
    IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba)
        : gravity_(gravity)
        , bg_(init_bg)
        , ba_(init_ba)
    {
    }
    void AddIMU(const IMU& imu);
    // 组成NavState
    NavStated getNavState() const { return NavStated(timestamp_, R_, p_, v_, bg_, ba_); }
    SO3 getR() const { return R_; }
    Vec3d getV() const { return v_; }
    Vec3d getP() const { return p_; }

private:
    // 累计量
    SO3 R_;
    Vec3d v_ = Vec3d::Zero();
    Vec3d p_ = Vec3d::Zero();

    double timestamp_ = 0.0;

    // 零偏，由外部设定
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    Vec3d gravity_ = Vec3d(0, 0, -9.8); // 重力
};

}