# Robot Algorithms

机器人算法实现库 - Robot algorithms implementation library

## 内容 (Contents)

本仓库包含多种机器人算法的实现，包括：
- ICP (Iterative Closest Point) 3D 点云配准
- NDT (Normal Distributions Transform) 3D 点云配准
- IMU 积分和导航状态估计
- 数学工具和实用函数

This repository contains implementations of various robot algorithms, including:
- ICP (Iterative Closest Point) 3D point cloud registration
- NDT (Normal Distributions Transform) 3D point cloud registration
- IMU integration and navigation state estimation
- Mathematical utilities and helper functions

## 文档 (Documentation)

### FAST-LIO2 变量符号说明
如果你想了解 FAST-LIO2 中 **x̂** (x 上有帽) 和 **x̄** (x 上有横线) 这两个变量的含义，请查看：

If you want to understand the meaning of **x̂** (x with hat) and **x̄** (x with bar) variables in FAST-LIO2, please see:

📖 **[docs/FASTLIO2_Variable_Notation.md](docs/FASTLIO2_Variable_Notation.md)**

该文档详细解释了：
- x̂: 预测状态 (Predicted/A priori state)
- x̄: 校正状态 (Corrected/A posteriori state)
- ESKF 工作流程和数学原理

The document provides detailed explanations of:
- x̂: Predicted/A priori state
- x̄: Corrected/A posteriori state  
- ESKF workflow and mathematical principles

## 构建 (Build)

```bash
mkdir build && cd build
cmake ..
make
```

## 依赖 (Dependencies)

- Eigen3
- PCL (Point Cloud Library)
- Sophus (Lie algebra library)
- glog (Logging library)
- gflags

## 项目结构 (Project Structure)

```
├── include/          # 头文件
│   ├── nav_state.h   # 导航状态定义
│   ├── imu.h         # IMU 数据结构
│   ├── imu_integration.h  # IMU 积分
│   ├── icp3d.h       # ICP 算法
│   └── ndt3d.h       # NDT 算法
├── src/              # 源文件实现
├── test/             # 测试代码
├── docs/             # 文档
├── notebooks/        # Jupyter notebooks (数学推导)
└── data/             # 数据文件
```

## 许可证 (License)

请参考项目根目录下的许可证文件。
