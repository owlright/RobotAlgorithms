#include "imu.h"
#include "imu_integration.h"
#include "io_utils.h"
#include <glog/logging.h>
#include <iomanip>

DEFINE_string(imu_txt_path, "../data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, false, "是否显示图形界面");

using ra::Vec3d;
using ra::IMU;

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_imu_txt_path.empty()) {
        LOG(ERROR) << "Please specify the IMU data file path using --imu_txt_path.";
    }
    ra::utils::io::TxtIO<ra::IMU> imuTxt(FLAGS_imu_txt_path);
    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    ra::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    imuTxt.SetParseFunc([](const std::string& line) -> ra::IMU {
        std::stringstream ss(line);
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU") {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            return ra::IMU(time, ra::Vec3d(gx, gy, gz), ra::Vec3d(ax, ay, az));
        } else {
            return ra::IMU(); // 返回一个默认的IMU对象
        }
    }).SetProcessFunc([&imu_integ](const ra::IMU& imu) {
        imu_integ.AddIMU(imu);
    });
    return 0;
}