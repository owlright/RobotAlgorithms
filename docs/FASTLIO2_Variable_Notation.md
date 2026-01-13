# FAST-LIO2 变量符号说明 (Variable Notation Guide)

## 概述 (Overview)

FAST-LIO2 使用 **ESKF (Error State Kalman Filter，误差状态卡尔曼滤波器)** 进行状态估计。在 ESKF 框架中，两个重要的状态变量符号分别是 **x̂** 和 **x̄**，它们在滤波过程中扮演不同的角色。

FAST-LIO2 uses **ESKF (Error State Kalman Filter)** for state estimation. In the ESKF framework, two important state variable notations are **x̂** and **x̄**, which play different roles in the filtering process.

---

## 变量含义 (Variable Meanings)

### 1. x̂ (x with hat/caret) - 预测状态 (Predicted State)

**x̂** 表示 **先验状态估计** 或 **预测状态**，是在卡尔曼滤波的**预测步骤 (Prediction Step)** 中得到的状态。

- **英文**: A priori state estimate / Predicted state
- **来源**: 通过上一时刻的状态和运动模型（如 IMU 积分）预测得到
- **特点**: 
  - 仅使用运动模型（Process Model）
  - 未融合观测数据（Measurement）
  - 包含预测的不确定性

**数学表示**:
```
x̂ₖ = f(x̄ₖ₋₁, uₖ)
```
其中：
- `x̄ₖ₋₁`: 上一时刻的后验状态
- `uₖ`: 控制输入（如 IMU 数据）
- `f()`: 状态转移函数

**在 FAST-LIO2 中的应用**:
```cpp
// IMU 预测步骤 (Prediction step)
// 更新位置、速度、姿态
p_pred = p + v * dt + 0.5 * gravity * dt * dt + 0.5 * (R * (acce - ba)) * dt * dt;
v_pred = v + R * (acce - ba) * dt + gravity * dt;
R_pred = R * SO3::exp((gyro - bg) * dt);
```

---

### 2. x̄ (x with bar/overline) - 校正状态 (Corrected State)

**x̄** 表示 **后验状态估计** 或 **校正状态**，是在卡尔曼滤波的**更新步骤 (Update Step)** 中，融合观测数据后得到的最优状态估计。

- **英文**: A posteriori state estimate / Corrected state
- **来源**: 通过预测状态 x̂ 和观测数据融合得到
- **特点**:
  - 融合了预测和观测信息
  - 是当前时刻的最优估计
  - 不确定性比预测状态更小

**数学表示**:
```
x̄ₖ = x̂ₖ + Kₖ(zₖ - h(x̂ₖ))
```
其中：
- `x̂ₖ`: 预测状态
- `Kₖ`: 卡尔曼增益 (Kalman Gain)
- `zₖ`: 观测数据（如激光雷达点云）
- `h()`: 观测模型

**在 FAST-LIO2 中的应用**:
```cpp
// 激光雷达更新步骤 (Update step with LiDAR)
// 计算卡尔曼增益
K = P_pred * H^T * (H * P_pred * H^T + R)^(-1);

// 状态更新
x_corrected = x_pred + K * (z - h(x_pred));

// 协方差更新
P_corrected = (I - K * H) * P_pred;
```

---

## ESKF 工作流程 (ESKF Workflow)

ESKF 的完整循环包括两个主要步骤：

### 步骤 1: 预测 (Prediction)
使用 IMU 数据进行状态预测，得到 **x̂**:

```
时间: t(k-1) → t(k)
输入: x̄(k-1), IMU(k)
输出: x̂(k), P̂(k)

状态预测: x̂(k) = f(x̄(k-1), IMU(k))
协方差预测: P̂(k) = F * P̄(k-1) * F^T + Q
```

### 步骤 2: 更新 (Update)
使用激光雷达等传感器数据进行状态校正，得到 **x̄**:

```
时间: t(k)
输入: x̂(k), P̂(k), LiDAR(k)
输出: x̄(k), P̄(k)

卡尔曼增益: K(k) = P̂(k) * H^T * (H * P̂(k) * H^T + R)^(-1)
状态更新: x̄(k) = x̂(k) + K(k) * (z(k) - h(x̂(k)))
协方差更新: P̄(k) = (I - K(k) * H) * P̂(k)
```

---

## 状态向量组成 (State Vector Components)

在 FAST-LIO2 中，状态向量 x 通常包括：

```cpp
x = [R, p, v, bg, ba]^T
```

其中：
- **R**: 旋转矩阵 (Rotation, SO(3))
- **p**: 位置 (Position, R³)
- **v**: 速度 (Velocity, R³)
- **bg**: 陀螺仪零偏 (Gyroscope bias)
- **ba**: 加速度计零偏 (Accelerometer bias)

对应的代码实现见 `include/nav_state.h`:
```cpp
template <typename T>
struct NavState {
    SO3 R_;                   // 旋转
    Vec3 p_ = Vec3::Zero();   // 平移
    Vec3 v_ = Vec3::Zero();   // 速度
    Vec3 bg_ = Vec3::Zero();  // gyro 零偏
    Vec3 ba_ = Vec3::Zero();  // acce 零偏
};
```

---

## 符号对照表 (Notation Summary)

| 符号 | 英文名称 | 中文名称 | 含义 | 获得方式 |
|------|---------|---------|------|---------|
| x̂ | x-hat | x 带尖帽 | 预测状态/先验估计 | 通过运动模型预测 |
| x̄ | x-bar | x 带横线 | 校正状态/后验估计 | 通过融合观测数据更新 |
| P̂ | P-hat | P 带尖帽 | 预测协方差 | 协方差预测 |
| P̄ | P-bar | P 带横线 | 校正协方差 | 协方差更新 |
| K | Kalman Gain | 卡尔曼增益 | 融合权重 | K = P̂H^T(HP̂H^T + R)^(-1) |

---

## 直观理解 (Intuitive Understanding)

可以这样理解：

1. **x̂ (预测)**：根据"我之前在哪里"和"我怎么运动的"来推测"我现在应该在哪里"
   - 依赖于 IMU 的积分
   - 会随时间累积误差

2. **x̄ (校正)**：结合"我推测的位置"和"传感器实际观测到的位置"来确定"我真正在哪里"
   - 融合了激光雷达等外部观测
   - 修正了累积误差
   - 是最终输出的状态

---

## 参考资料 (References)

1. Xu, W., & Zhang, F. (2021). FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter. IEEE Robotics and Automation Letters.

2. Xu, W., Cai, Y., He, D., Lin, J., & Zhang, F. (2022). FAST-LIO2: Fast Direct LiDAR-inertial Odometry. IEEE Transactions on Robotics.

3. Solà, J. (2017). Quaternion kinematics for the error-state Kalman filter. arXiv preprint arXiv:1711.02508.

---

## 相关代码 (Related Code)

- **IMU 预测**: `src/imu_integration.cpp` - `IMUIntegration::AddIMU()`
- **状态定义**: `include/nav_state.h` - `NavState` 结构体
- **数学推导**: `notebooks/refer.ipynb` - SO(3) 指数映射和李代数相关
