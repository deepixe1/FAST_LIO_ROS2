# 07. 二次开发与代码扩展实战指南

对于准备深入修改 FAST-LIO2 算法（例如添加新传感器、修改状态向量、优化残差函数、调整硬件平台）的开发者而言，本项目有若干强耦合的“隐形规则”与工程细节。若直接修改代码而忽略这些细节，极易引发编译报错（模板元编程不匹配）、运行时崩溃（Eigen 内存对齐）、甚至位姿漂移。

本章节汇总了最常见、最具实用价值的二次开发实战指南与避坑手册。

---

## 1. 如何修改或扩展状态向量

默认的状态向量切空间维度为 23 维（位置 3 + 姿态 3 + 外参旋转 3 + 外参平移 3 + 速度 3 + 陀螺零偏 3 + 加计零偏 3 + 重力 2）。

### 场景示例：新增轮速计比例因子或时间偏差
若你希望新增状态量（例如 1 维轮速计刻度系数 $s_{wheel}$，或 3 维未知杆臂外参）：

#### 步骤 1：在 `include/use-ikfom.hpp` 中扩展宏定义
修改 `MTK_BUILD_MANIFOLD`（[include/use-ikfom.hpp#L12-L21](../include/use-ikfom.hpp#L12-L21)）：
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
((vect1, scale_wheel)) // 新增状态量：维度 1
);
```

#### 步骤 2：调整微分与雅可比函数
1. **状态微分 `get_f`**：
   原返回矩阵维度为 $24 \times 1$，现平坦维度变为 $25 \times 1$。
   若新状态导数为 0（常值假设），则设对应位置为 0：
   ```cpp
   Eigen::Matrix<double, 25, 1> res = Eigen::Matrix<double, 25, 1>::Zero();
   // 填充前 24 维 ...
   res(24) = 0.0; // scale_wheel 导数
   ```
2. **状态转移雅可比 `df_dx`**：
   返回矩阵尺寸由 $24 \times 23$ 改为 $25 \times 24$。
3. **过程噪声驱动矩阵 `df_dw`**：
   若该状态受随机游走噪声驱动，相应扩展噪声维度。

#### 步骤 3：修改 `esekfom.hpp` 中的硬编码分块（重要！）
> [!CAUTION]
> **源码隐藏陷阱：12 维分块硬编码**
> 在 [include/IKFoM_toolkit/esekfom/esekfom.hpp#L1647](../include/IKFoM_toolkit/esekfom/esekfom.hpp#L1647) 与 [L1784-L1809](../include/IKFoM_toolkit/esekfom/esekfom.hpp#L1784-L1809) 中，作者为了极致的加速，硬编码了 `12` 这个常数：
> ```cpp
> Eigen::Matrix<scalar_type, Eigen::Dynamic, 12> h_x_ = dyn_share.h_x;
> Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_;
> P_temp.template block<12, 12>(0, 0) += HTH;
> ```
> 这是因为原点面残差仅与前 12 个状态（位置、姿态、雷达外参）相关。
> **如果你修改了状态的顺序，或者新增了会被点面残差直接影响的状态量，必须同步修改此处的 `12` 为新的分块维度，否则求逆更新将被截断！**

---

## 2. 如何接入外部传感器（如 GNSS、轮速计）

在真实机器人系统中，单纯的激光惯导在长直走廊（几何退化）或空旷农田（缺乏点云反射）可能产生发散。接入轮速计或 GNSS 是最常见的改造需求。

FAST-LIO2 的 IESKF 架构天然支持**多步观测更新（Multi-update）**：

### 模式 A：在主循环中追加独立更新步骤（推荐）
在每帧雷达点云更新之前或之后，可以根据外部传感器的时间戳，单独调用卡尔曼更新：

```cpp
// 示例：引入外部绝对位置测量 z_pos (如 GNSS RTK)
void update_gnss(const Eigen::Vector3d &z_pos, const Eigen::Matrix3d &R_gnss)
{
    state_ikfom s = kf.get_x();
    auto P = kf.get_P();
    
    // 残差 r = z - h(x)
    Eigen::Vector3d residual = z_pos - s.pos;
    
    // 观测雅可比 H_gnss (3 x 23)，仅在位置分量上有单位阵 I
    Eigen::Matrix<double, 3, 23> H_gnss = Eigen::Matrix<double, 3, 23>::Zero();
    H_gnss.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    
    // 计算卡尔曼增益 (维度 23x3，直接求逆 3x3 矩阵即可)
    Eigen::Matrix3d S = H_gnss * P * H_gnss.transpose() + R_gnss;
    Eigen::Matrix<double, 23, 3> K = P * H_gnss.transpose() * S.inverse();
    
    // 状态与协方差更新
    Eigen::Matrix<double, 23, 1> dx = K * residual;
    s.boxplus(dx);
    kf.change_x(s);
    kf.change_P((Eigen::Matrix<double, 23, 23>::Identity() - K * H_gnss) * P);
}
```

### 模式 B：在 `h_share_model` 中联合追加残差行
如果传感器频率与雷达同步，可以在 `h_share_model`（[src/laserMapping.cpp#L677](../src/laserMapping.cpp#L677)）构建雅可比时，将外部约束直接拼接到 `ekfom_data.h` 和 `ekfom_data.h_x` 的末尾，由 IESKF 一并联合迭代求解。

---

## 3. 点面残差计算与拟合算法优化

点面拟合核心实现于 [include/common_lib.h#L224-L255](../include/common_lib.h#L224-L255) 的 `esti_plane` 函数中。

### 3.1 增加平面拟合的鲁棒性
原始代码使用固定 5 个最近邻点（`NUM_MATCH_POINTS = 5`）求解超定线性方程 $Ax = b$：
```cpp
Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
```
- **点数扩展**：若点云噪声较大（如低成本固态雷达），建议将 `NUM_MATCH_POINTS` 扩大为 7 或 9，以提升法向量解算精度；
- **平面有效性阈值判断**：
  [src/laserMapping.cpp#L717](../src/laserMapping.cpp#L717)：
  ```cpp
  esti_plane(pabcd, points_near, 0.1f);
  ```
  `0.1f`（10 cm）代表拟合残差的最大容许误差。如果用于室内高精度激光建图，可缩减至 `0.03f` ~ `0.05f`，滤除边缘毛刺；在野外植被茂密场景，可适度放宽至 `0.15f` ~ `0.2f`。

---

## 4. 关键调参指南（针对不同机器人运动场景）

| 工况场景 | 核心问题 | 推荐调整参数及修改方案 |
| :--- | :--- | :--- |
| **四足机器人 / 履带车** | 机身剧烈机械高频震颤，IMU 噪声剧增 | 1. 将 `mapping.acc_cov` 从 0.1 调大到 1.0 ~ 5.0<br/>2. 将 `mapping.gyr_cov` 调大到 0.5 ~ 1.0<br/>3. 将 `point_filter_num` 调为 2（保证更多几何点来约束位姿） |
| **高速竞速无人机** | 角速度极大，线速度高，扫描期畸变严重 | 1. 确保 IMU 输出频率 $\ge 400\text{ Hz}$<br/>2. 将 `max_iteration` 从 3 增加到 4 或 5<br/>3. 必须精准标定雷达与 IMU 外参，并将 `mapping.extrinsic_est_en` 设为 `false` |
| **长直隧道 / 走廊** | 前后方向缺乏几何约束，容易沿前进方向漂移 | 1. 检测雅可比信息矩阵 $H^T H$ 最小特征值<br/>2. 引入轮速计或双目视觉约束融合 |
| **超远距离室外开阔地** | 点云随距离发散，稀疏度高 | 1. 调大 `mapping.det_range`（如 450 米）<br/>2. 调大 `filter_size_surf` 到 0.5 ~ 0.8 米<br/>3. 调大 `cube_side_length` 到 1000 ~ 2000 米 |

---

## 5. 工程开发致命陷阱排查清单

在修改此仓库时，请随时对照以下清单自检：

### 1. Eigen 内存对齐崩溃（`Segmentation Fault`）
- **现象**：程序一运行或一进入回调就报段错误 `SIGSEGV`，GDB 堆栈停留在 Eigen 向量初始化处。
- **原因**：类中包含 Eigen 固定尺寸矩阵（如 `Vector3d`, `Matrix4d`）但未启用对齐。
- **解决办法**：
  在所有包含 Eigen 成员的类或结构体 `public` 区域首行加入：
  ```cpp
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ```
  在包含此类作为 `std::vector` 容器元素时，必须使用内存对齐分配器：
  ```cpp
  std::vector<MyStruct, Eigen::aligned_allocator<MyStruct>> my_vector;
  ```

### 2. 点级时间戳与单位不匹配
- 确保输入到 `UndistortPcl` 的点云，其每个点的 `curvature` 字段存储的都是**相对该雷达帧起始时间的毫秒数（ms）**；
- 确保点云在进入去畸变前经过了按时间升序排序（`std::sort(..., time_list)`）。

### 3. 多线程互斥锁死锁
- 在 `standard_pcl_cbk`、`livox_pcl_cbk` 和 `imu_cbk` 中访问 `lidar_buffer` 与 `imu_buffer` 时均使用了 `mtx_buffer`；
- 如果你在这些回调内增加了耗时的算法逻辑或阻塞式打印，会导致定时器 `timer_callback` 拿不到锁而发生严重丢帧。**原则：回调内仅做必要的数据入队，耗时处理全部交给主线程！**
