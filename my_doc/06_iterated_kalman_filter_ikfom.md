# 06. IKFoM 流形滤波框架深度解析

FAST-LIO2 的滤波求解并没有采用手写冗长矩阵乘法的传统方式，而是依托香港大学 MaRS 实验室开源的通用流形滤波工具箱 **IKFoM（Iterated Kalman Filter on Manifolds）**。

IKFoM 的源码位于 [include/IKFoM_toolkit](../include/IKFoM_toolkit)，应用适配层位于 [include/use-ikfom.hpp](../include/use-ikfom.hpp)。

本章节深入剖析：宏定义元编程机制、切空间自动微积分算子、以及 `update_iterated_dyn_share_modified` 迭代更新核心函数的逐行实现。

---

## 1. 流形状态声明与元编程宏机制

在 [include/use-ikfom.hpp#L12-L21](../include/use-ikfom.hpp#L12-L21) 中，系统状态仅用数行宏代码便完成声明：

```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);
```

### 1.1 `MTK_BUILD_MANIFOLD` 展开后的内部构造
该宏通过 C++ 模板元编程技术，为 `state_ikfom` 类自动生成了以下关键方法与类型定义：
1. **维度常数**：
   - `DOF`（自由度，即切空间误差维度）：$3 + 3 + 3 + 3 + 3 + 3 + 3 + 2 = 23$；
   - `DIM`（内部平坦存储维度）：$3 + 3 + 3 + 3 + 3 + 3 + 3 + 3 = 24$；
2. **算子重载**：
   - `boxplus(const Matrix<double, 23, 1> &vec)`：对各个子流形分别执行 $\boxplus$，例如 `pos += vec.segment<3>(0)`、`rot = rot * Exp(vec.segment<3>(3))`；
   - `boxminus(Matrix<double, 23, 1> &vec, const state_ikfom &other)`：计算两流形之间的切空间差向量 $\delta x$；
3. **子流形索引与雅可比生成**：
   自动记录旋转子流形在误差状态向量中的偏移起始下标（如 `rot` 起始于 3，`offset_R_L_I` 起始于 6，`grav` 起始于 21）。

---

## 2. 状态微分与过程雅可比矩阵实现

在 [include/use-ikfom.hpp](../include/use-ikfom.hpp) 中，用户只需要提供以下三个回调函数给 IKFoM 框架：

### 2.1 连续系统微分函数 `get_f`
```cpp
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);              // omega = in.gyro - s.bg
    vect3 a_inertial = s.rot * (in.acc - s.ba); // 世界系下的线加速度
    
    for(int i = 0; i < 3; i++ ){
        res(i) = s.vel[i];                     // 1. 位置导数 = 速度
        res(i + 3) = omega[i];                 // 2. 朝向切空间角速度
        res(i + 12) = a_inertial[i] + s.grav[i];// 3. 速度导数 = 比力 + 重力
    }
    return res;
}
```

### 2.2 状态雅可比矩阵 `df_dx`
返回尺寸为 $24 \times 23$ 的分块矩阵：
```cpp
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity(); // dp / dv
    
    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_); // dv / d_theta
    cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();                  // dv / d_ba
    
    Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 2>(12, 21) = grav_matrix;                                // dv / d_grav (S2流形)
    
    cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();                // d_theta / d_bg
    return cov;
}
```

### 2.3 过程噪声驱动矩阵 `df_dw`
尺寸为 $24 \times 12$：
```cpp
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();   // dv / d_na
    cov.template block<3, 3>(3, 0)  = -Eigen::Matrix3d::Identity(); // d_theta / d_ng
    cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();  // dbg / d_nbg
    cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();  // dba / d_nba
    return cov;
}
```

---

## 3. 核心迭代求解函数：`update_iterated_dyn_share_modified`

本函数实现于 [include/IKFoM_toolkit/esekfom/esekfom.hpp#L1619-L1840](../include/IKFoM_toolkit/esekfom/esekfom.hpp#L1619-L1840)，是整个 FAST-LIO2 算法运算耗时最集中、数学逻辑最精密的函数。

### 3.1 变量快照与初始化
```cpp
dyn_share_datastruct<scalar_type> dyn_share;
state x_propagated = x_;      // 备份前向积分推算的名义先验状态 (prior)
cov P_propagated = P_;        // 备份先验协方差矩阵 (23x23)
vectorized_state dx_new = vectorized_state::Zero();
```

### 3.2 迭代更新大循环（Gauss-Newton Loop）
```cpp
for(int i = -1; i < maximum_iter; i++)
{
    dyn_share.valid = true;
    // 1. 调用外部观测模型 (即 laserMapping.cpp 中的 h_share_model)
    // 内部完成雷达点的最近邻搜索、平面拟合，输出残差 dyn_share.h 与雅可比 dyn_share.h_x
    h_dyn_share(x_, dyn_share);

    if(!dyn_share.valid) continue;

    Eigen::Matrix<scalar_type, Eigen::Dynamic, 12> h_x_ = dyn_share.h_x;
    dof_Measurement = h_x_.rows(); // 参与约束的有效特征点数 m
    
    // 2. 计算当前迭代状态与先验状态在切空间上的偏置
    vectorized_state dx;
    x_.boxminus(dx, x_propagated);
    dx_new = dx;

    // 3. 伴随变换校正流形切空间的先验协方差 P
    P_ = P_propagated;
    // 对 SO3 旋转部分通过右雅可比伴随矩阵进行旋转基底变换
    ...
```

### 3.3 逆向维度求逆核心段（加速精髓）
由于 $m \gg 23$，代码直接进入加速求逆分支：
```cpp
// 1. 先验协方差归一化求逆: (P_ / R)^(-1)
cov P_temp = (P_ / R).inverse();

// 2. 累加观测信息矩阵 H^T * H (仅 12x12 有值)
Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_; 
P_temp.template block<12, 12>(0, 0) += HTH;

// 3. 仅对 23x23 的矩阵执行求逆操作
cov P_inv = P_temp.inverse(); // 仅对 23x23 矩阵求逆！

// 4. 增益项计算
// K_h = (H^T H + sigma^2 P^(-1))^(-1) * H^T * h
K_h = P_inv.template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h;

// K_x = (H^T H + sigma^2 P^(-1))^(-1) * H^T * H
K_x.setZero();
K_x.template block<n, 12>(0, 0) = P_inv.template block<n, 12>(0, 0) * HTH;
```

### 3.4 状态增量合成与流形叠加
```cpp
// 计算最终综合步长 dx_
// dx_ = K_h + (K_x - I) * (x_k \boxminus x_prior)
Matrix<scalar_type, n, 1> dx_ = K_h + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new; 

// 流形上的加法更新
x_.boxplus(dx_);
```

### 3.5 收敛判据检查
```cpp
dyn_share.converge = true;
for(int i = 0; i < n; i++)
{
    // 如果任何一个状态维度的更新步长大于阈值 limit[i] (即 epsi[i] = 0.001)
    if(std::fabs(dx_[i]) > limit[i])
    {
        dyn_share.converge = false;
        break;
    }
}
// 若收敛，或者达到最大迭代次数，循环终止
```

---

## 4. 后验协方差更新

在迭代完全收敛后，最后一次计算得到的 `P_inv` 即为最新的后验估计协方差矩阵：
$$P_{\mathrm{post}} = (H^T R^{-1} H + P^{-1})^{-1}$$
在代码中：
```cpp
P_ = P_inv;
```
这一后验协方差矩阵将被传递给下一扫描周期的 IMU 前向传播过程，并同步提取前 6 个自由度的位姿方差填充至 `/Odometry` 话题的 `pose.covariance` 字段中（见 [src/laserMapping.cpp#L635-L645](../src/laserMapping.cpp#L635-L645)）。
