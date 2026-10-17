# 03. IMU 处理与点云畸变补偿

IMU 预处理与点云去畸变是激光惯导里程计（LIO）能否在高动态工况下保持高精度的分水岭。FAST-LIO2 的这一部分完整封装在 [src/IMU_Processing.hpp](../src/IMU_Processing.hpp) 中，核心类为 `ImuProcess`。

本章节深度拆解：静止初始化、IMU 前向状态与协方差推算、以及基于反向传播的点云运动畸变补偿算法。

---

## 1. 静止初始化流程 (`IMU_init`)

当系统刚启动时，机器人需要处于相对静止状态（通常为前 10 帧点云到达的时间窗口，对应代码中的 `MAX_INI_COUNT = 10`）。

### 1.1 初始化目标
1. **重力加速度方向对齐**：通过测量静止状态下的重力向量确定世界系下的初始倾角（Pitch 与 Roll）；
2. **陀螺仪初始零偏估计**：静止状态下陀螺仪的角速度读数即为零偏 $b_g$；
3. **测量噪声协方差标定**：在线统计线加速度与角速度的样本方差；
4. **初始滤波器状态与协方差赋值**。

### 1.2 在线增量均值与方差算法
为了避免缓存海量 IMU 历史数据，代码采用了 Welford 增量更新公式（见 [src/IMU_Processing.hpp#L182-L190](../src/IMU_Processing.hpp#L182-L190)）：

$$\mu_N = \mu_{N-1} + \frac{x_N - \mu_{N-1}}{N}$$
$$\sigma_N^2 = \frac{N-1}{N} \sigma_{N-1}^2 + \frac{N-1}{N^2} (x_N - \mu_{N-1}) \odot (x_N - \mu_{N-1})$$

```cpp
mean_acc += (cur_acc - mean_acc) / N;
mean_gyr += (cur_gyr - mean_gyr) / N;

cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
```

### 1.3 初始状态与协方差矩阵注入
统计结束后（`init_iter_num > MAX_INI_COUNT`）：
1. **重力向量初始化**（[src/IMU_Processing.hpp#L193](../src/IMU_Processing.hpp#L193)）：
   由于加速度计测量的是比力（Specific Force）：$f = a - g$，静止时 $a = 0 \implies f = -g$。
   因此真实重力方向为测量比力的反方向：
   $$g_{init} = - \frac{\bar{a}}{\|\bar{a}\|} \cdot 9.81$$
2. **零偏与外参初始化**：
   $$b_g = \bar{\omega}, \quad b_a = 0$$
   外参 $R_L^I, p_L^I$ 载入 YAML 配置文件给定值；
3. **先验协方差矩阵 $P_0$ 设置**（[src/IMU_Processing.hpp#L201-L208](../src/IMU_Processing.hpp#L201-L208)）：
   - 外参平移与旋转先验协方差：$10^{-5}$（初始信任离线标定结果）；
   - 速度协方差：$10^{-4}$；
   - 零偏协方差：$10^{-3}$；
   - 重力方向协方差：$10^{-5}$。

---

## 2. IMU 前向传播（中值数值积分）

在两帧激光扫描之间，IMU 会以较高频率（通常 200 Hz ~ 1000 Hz）输出一系列测量。FAST-LIO2 遍历这些数据，驱动 IESKF 执行前向名义状态与协方差预测。

### 2.1 跨帧数据连续性保证
在函数 `UndistortPcl` 开头：
```cpp
auto v_imu = meas.imu;
v_imu.push_front(last_imu_); // 关键：插入上一帧末尾的 IMU 测量
```
这保证了时间区间 $[t_{last\_scan\_end}, t_{cur\_scan\_end}]$ 的完全连续。

### 2.2 中值积分方程
对于相邻两个 IMU 读数 $(head, tail)$，时间步长 $\Delta t = t_{tail} - t_{head}$，取中值角速度与线加速度：
$$\bar{\omega} = \frac{\omega_{head} + \omega_{tail}}{2}$$
$$\bar{a} = \frac{a_{head} + a_{tail}}{2} \cdot \frac{9.81}{\|\bar{a}_{init}\|}$$
调用流形滤波预测接口（[src/IMU_Processing.hpp#L278](../src/IMU_Processing.hpp#L278)）：
```cpp
in.acc = acc_avr;
in.gyro = angvel_avr;
kf_state.predict(dt, Q, in);
```

### 2.3 IMU 位姿历史轨迹缓存 (`IMUpose`)
为了后续对每个激光反射点进行精确插值，算法在每次前向积分后，将该时刻 IMU 的状态记录到 `IMUpose` 容器中：
```cpp
/* 记录相对于雷达帧起始时刻的时间差 offs_t */
double offs_t = tail_stamp - pcl_beg_time;
IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, 
                             imu_state.vel, imu_state.pos, 
                             imu_state.rot.toRotationMatrix()));
```
其中 `acc_s_last` 为世界系下的总加速度：
$$a_w = R \cdot (\bar{a} - b_a) + g$$
`angvel_last` 为扣除零偏后的真实角速度：
$$\omega_{clean} = \bar{\omega} - b_g$$

### 2.4 雷达帧尾尾部对齐
如果雷达帧结束时间 $t_{lidar\_end}$ 与最后一个 IMU 测量时间 $t_{imu\_end}$ 不完全一致，通过微小步长额外预测一次：
```cpp
double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
dt = note * (pcl_end_time - imu_end_time);
kf_state.predict(dt, Q, in);
```
至此，滤波器的当前状态 $\hat{x}$ 已精确对齐到**当前扫描帧的结束时刻**（$t_e$）。

---

## 3. 点云运动畸变反向补偿算法 (`UndistortPcl`)

### 3.1 运动畸变原理与基准选择
激光雷达（不论是机械旋转式还是固态棱镜式）在完成一帧扫描（通常耗时 100 ms）的过程中，载体自身在高速平移和旋转。因此，点云中每一个点是在不同的时刻采集的，如果把它们直接当成同一瞬时的刚体点云，会导致地图边缘重影或发散。

> [!TIP]
> **为什么对齐到帧尾时刻 $t_e$ 而不是帧头？**
> 因为 IMU 前向推算自然终止在帧尾 $t_e$。如果对齐到帧尾，当前激光帧被矫正后的瞬时位姿恰好等于 IESKF 最新的先验状态 $\hat{x}_e$。在后续残差计算和地图更新时，无需再进行额外的位姿时间回退转换。

```
时间轴演进:
  t_beg ---------------------- t_i (第i个点) ---------------------- t_end (帧尾)
    |                             |                                  |
 [IMU状态]                    [位姿插值]                        [滤波先验对齐]
```

### 3.2 严密几何坐标变换推导

设某激光点采集时刻为 $t_i \in [t_{head}, t_{tail}]$，该点在激光雷达本体坐标系下的读数为 ${}^L p_i$。

#### 步骤 1：时间插值出 $t_i$ 时刻的位姿
利用与其相邻的前后两个 IMU 关键帧 $(head, tail)$，设 $\Delta t = t_i - t_{head}$：
$$R_i = R_{head} \cdot \mathrm{Exp}(\omega_{tail} \cdot \Delta t)$$
$$p_i = p_{head} + v_{head} \Delta t + \frac{1}{2} a_{tail} \Delta t^2$$

#### 步骤 2：计算该点在世界系下的绝对坐标 ${}^W p$
利用外参 $(R_L^I, p_L^I)$ 将雷达点投影到 $t_i$ 时刻的 IMU 系，再投影到世界系：
$${}^W p = R_i \cdot \left( R_L^I \cdot {}^L p_i + p_L^I \right) + p_i$$

#### 步骤 3：反向投影到帧尾雷达坐标系 ${}^{L_e} p$
在帧尾时刻 $t_e$，载体 IMU 位姿为 $(R_e, p_e)$，雷达外参不变。我们要求寻找一个虚拟点 ${}^{L_e} p$（即去畸变后的点坐标），使得若在 $t_e$ 时刻发射激光，该点映射到世界坐标应与真实物理点 ${}^W p$ 完全重合：
$$R_e \cdot \left( R_L^I \cdot {}^{L_e} p + p_L^I \right) + p_e = {}^W p$$

求解 ${}^{L_e} p$：
$$R_L^I \cdot {}^{L_e} p + p_L^I = R_e^T \left( {}^W p - p_e \right)$$
$$R_L^I \cdot {}^{L_e} p + p_L^I = R_e^T \left( R_i (R_L^I \cdot {}^L p_i + p_L^I) + (p_i - p_e) \right)$$
$${}^{L_e} p = (R_L^I)^T \left[ R_e^T \left( R_i (R_L^I \cdot {}^L p_i + p_L^I) + \underbrace{(p_i - p_e)}_{T_{ei}} \right) - p_L^I \right]$$

### 3.3 源码实现细节（[src/IMU_Processing.hpp#L303-L336](../src/IMU_Processing.hpp#L303-L336)）

```cpp
auto it_pcl = pcl_out.points.end() - 1;

// 从后往前反向遍历 IMU 轨迹
for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
{
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);

    // 匹配属于当前 (head, tail) 时间区间内的所有激光点
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        
        // T_ei = p_i - p_e (在世界系下的位移差)
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        
        // 执行反向投影公式
        V3D P_compensate = imu_state.offset_R_L_I.conjugate() * 
                           (imu_state.rot.conjugate() * 
                           (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) 
                           - imu_state.offset_T_L_I);
        
        // 覆写原点云坐标为去畸变后的坐标
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) break;
    }
}
```

> [!IMPORTANT]
> **双指针反向扫描设计精妙性**：
> 原始点云先按时间戳正序排序（`sort(..., time_list)`）。如果用正向循环，每个点都要在 `IMUpose` 中做二分查找或线性搜索；而作者采用**点云指针与 IMU 轨迹指针同时从后往前扫描**，单次遍历即可完成全帧万级点云的高效去畸变，时间复杂度严格为 $O(N_{pts} + N_{imu})$，在 CPU 上仅耗时 1~2 ms！

---

## 4. 二次开发与修改要点

如果你需要修改 IMU 处理逻辑，请特别注意以下约束：

1. **修改点云存储格式时的陷阱**：
   `it_pcl->curvature` 被强制用来存储点相对帧头的毫秒时间偏移（单位：ms）。如果你的上游驱动没有设置这个值，或者格式为微秒、纳秒，`dt` 计算将发生严重错误，去畸变点云会发散成散射面！
2. **高冲击振动场景（如机载起降、履带车）**：
   默认的前向传播为一阶欧拉中值积分。在剧烈高频振动下，加速度计高斯白噪声方差 `mapping.acc_cov` 必须从 0.1 调大到 1.0 甚至 5.0，否则 IESKF 会过于信任 IMU 高频振动导致的积分发散，使点面匹配残差急剧上升。
3. **静止初始化判定扩展**：
   当前初始化仅依赖固定计数器 `init_iter_num > 10`。如果需要支持动态开机初始化（即开机时小车已经在运动），需要替换该模块为动态对准算法（如基于预积分的加速度与重力旋转求解）。
