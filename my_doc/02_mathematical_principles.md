# 02. 数学基础与 IESKF 滤波理论

FAST-LIO2 的精髓在于其将连续时间运动学、流形几何（Manifold Theory）与迭代误差状态扩展卡尔曼滤波（Iterated Error-State Extended Kalman Filter, IESKF）无缝融合，并提出了**观测维度到状态维度的矩阵求逆转换技巧**，从而实现了超高频率（> 100 Hz）的激光惯导紧耦合位姿解算。

本章节将从零推导完整的数学体系，并与项目中的核心代码实现逐一对应。

---

## 1. 流形与李群李代数基础

在 3D 空间中，机器人的朝向属于特殊正交群 $SO(3)$，重力方向属于单位球面 $S^2$。由于这些状态不满足欧氏空间的线性加法，强行将其当作向量更新会导致奇异点（如欧拉角死锁）或破环正交约束。

### 1.1 旋转群 $SO(3)$ 与切空间 $\mathfrak{so}(3)$
对于旋转矩阵 $R \in SO(3)$，其对应的李代数向量 $\phi \in \mathbb{R}^3 \cong \mathfrak{so}(3)$。
- **反对称矩阵算子 $(\cdot)^\wedge$**（代码定义于 [include/so3_math.h#L7](../include/so3_math.h#L7)）：
  $$v = \begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} \implies v^\wedge = \begin{bmatrix} 0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0 \end{bmatrix}$$
- **指数映射 $\mathrm{Exp}: \mathbb{R}^3 \rightarrow SO(3)$**（罗德里格斯公式，见 [include/so3_math.h#L18-L34](../include/so3_math.h#L18-L34)）：
  $$\mathrm{Exp}(\phi) = \exp(\phi^\wedge) = I + \frac{\sin \|\phi\|}{\|\phi\|} \phi^\wedge + \frac{1 - \cos \|\phi\|}{\|\phi\|^2} (\phi^\wedge)^2$$
- **对数映射 $\mathrm{Log}: SO(3) \rightarrow \mathbb{R}^3$**（见 [include/so3_math.h#L82-L87](../include/so3_math.h#L82-L87)）：
  $$\theta = \arccos\left(\frac{\mathrm{tr}(R)-1}{2}\right), \quad \mathrm{Log}(R) = \frac{\theta}{2\sin\theta}(R - R^T)^\vee$$

### 1.2 单位球面流形 $S^2$（重力向量建模）
重力加速度的大小在局部区域为常数 $g \approx 9.81 \text{ m/s}^2$，因此重力向量只有**2 个自由度的方向信息**（俯仰与滚转），其幅值是无信息增益的已知量。
若将重力视为欧氏空间 $\mathbb{R}^3$，滤波器会在重力模长上耗费无意义的协方差更新，甚至导致不收敛。
FAST-LIO2 引入流形 $S^2$：
$$S^2 = \left\{ v \in \mathbb{R}^3 \;\middle|\; \|v\| = 9.81 \right\}$$
切空间维度为 2。在源码中由 `MTK::S2<double, 98090, 10000, 1>` 定义（见 [include/use-ikfom.hpp#L8](../include/use-ikfom.hpp#L8)）。

### 1.3 广义加减算子 $\boxplus$ 与 $\boxminus$
为了在非线性流形上统一定义误差和导数，引入流形运算：
$$x \boxplus \delta x : \mathcal{M} \times \mathbb{R}^n \rightarrow \mathcal{M}$$
$$x_1 \boxminus x_2 : \mathcal{M} \times \mathcal{M} \rightarrow \mathbb{R}^n$$
对于复合流形，各子流形分别执行对应的加减：
- 欧氏量：$p \boxplus \delta p = p + \delta p$
- 旋转量：$R \boxplus \delta \theta = R \cdot \mathrm{Exp}(\delta \theta)$
- 误差定义：$\delta \theta = \mathrm{Log}(R_2^T R_1)$

---

## 2. 状态向量与连续时间运动学方程

### 2.1 状态向量定义
FAST-LIO2 将系统真实状态定义于 24 维的复合流形 $\mathcal{M}$ 上，但其**切空间（误差状态）自由度为 23**：

$$x \triangleq \begin{bmatrix} p \\ R \\ R_L^I \\ p_L^I \\ v \\ b_g \\ b_a \\ g \end{bmatrix} \in \underbrace{\mathbb{R}^3}_{p} \times \underbrace{SO(3)}_{R} \times \underbrace{SO(3)}_{R_L^I} \times \underbrace{\mathbb{R}^3}_{p_L^I} \times \underbrace{\mathbb{R}^3}_{v} \times \underbrace{\mathbb{R}^3}_{b_g} \times \underbrace{\mathbb{R}^3}_{b_a} \times \underbrace{S^2}_{g}$$

对应的切空间误差状态向量 $\delta x \in \mathbb{R}^{23}$ 为：
$$\delta x \triangleq \begin{bmatrix} \delta p^T & \delta \theta^T & \delta \theta_{ext}^T & \delta p_{ext}^T & \delta v^T & \delta b_g^T & \delta b_a^T & \delta g^T \end{bmatrix}^T$$
- $\delta p \in \mathbb{R}^3$：世界系下的位置误差；
- $\delta \theta \in \mathbb{R}^3$：载体朝向误差（右扰动）；
- $\delta \theta_{ext} \in \mathbb{R}^3, \delta p_{ext} \in \mathbb{R}^3$：IMU 与 LiDAR 之间的外参旋转与平移误差；
- $\delta v \in \mathbb{R}^3$：速度误差；
- $\delta b_g \in \mathbb{R}^3, \delta b_a \in \mathbb{R}^3$：陀螺仪与加速度计零偏误差；
- $\delta g \in \mathbb{R}^2$：重力方向误差（2 自由度）。

### 2.2 连续时间名义运动学方程
IMU 输出包含真实角速度、线加速度及噪声：
$$\omega_m = \omega + b_g + n_g$$
$$a_m = a_{imu} + b_a + n_a$$
其中 $n_g \sim \mathcal{N}(0, \mathrm{Cov}_g)$，$n_a \sim \mathcal{N}(0, \mathrm{Cov}_a)$ 为测量白噪声；零偏满足随机游走模型：$\dot{b}_g = n_{bg}, \dot{b}_a = n_{ba}$。

系统状态随时间的微分方程为（对应 [include/use-ikfom.hpp#L47-L59](../include/use-ikfom.hpp#L47-L59) 的 `get_f`）：
$$\dot{p} = v$$
$$\dot{R} = R \cdot (\omega_m - b_g - n_g)^\wedge$$
$$\dot{R}_L^I = 0$$
$$\dot{p}_L^I = 0$$
$$\dot{v} = R \cdot (a_m - b_a - n_a) + g$$
$$\dot{b}_g = n_{bg}$$
$$\dot{b}_a = n_{ba}$$
$$\dot{g} = 0$$

---

## 3. 误差状态线性化方程与传播

为了构建卡尔曼滤波器的先验预测，对名义状态引入扰动：
$$x = \hat{x} \boxplus \delta x$$
利用一阶泰勒展开，得到误差状态的连续时间微分方程：
$$\dot{\delta x}(t) = F_x \delta x(t) + F_w w(t)$$
其中 $w = \begin{bmatrix} n_g^T & n_a^T & n_{bg}^T & n_{ba}^T \end{bmatrix}^T \in \mathbb{R}^{12}$。

### 3.1 状态转移矩阵 $F_x$ 推导
以速度误差微分 $\dot{\delta v}$ 为例：
$$\dot{\hat{v}} + \dot{\delta v} = (R \mathrm{Exp}(\delta \theta)) (a_m - \hat{b}_a - \delta b_a - n_a) + (\hat{g} + \delta g)$$
利用展开近似 $\mathrm{Exp}(\delta \theta) \approx I + \delta \theta^\wedge$：
$$\dot{\delta v} \approx - \hat{R} (a_m - \hat{b}_a)^\wedge \delta \theta - \hat{R} \delta b_a - \hat{R} n_a + \delta g$$

综合各项，连续时间系统雅可比矩阵 $F_x \in \mathbb{R}^{24 \times 23}$（代码中 `df_dx` 实现于 [include/use-ikfom.hpp#L61-L77](../include/use-ikfom.hpp#L61-L77)）：
$$F_x = \begin{bmatrix}
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & I_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & -I_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{3\times 3} & -\hat{R}(a_m - \hat{b}_a)^\wedge & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & -\hat{R} & M_g \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 2} \\
0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 2}
\end{bmatrix}$$
> 注：$M_g \in \mathbb{R}^{3 \times 2}$ 为单位球面切空间投影基矩阵（代码中由 `s.S2_Mx(grav_matrix, ...)` 给出）。

### 3.2 离散化协方差传播
在区间 $[t_k, t_{k+1}]$ 内，步长 $\Delta t = t_{k+1} - t_k$，采用一阶离散化：
$$\Phi_k = I + F_x \Delta t$$
协方差预测公式为：
$$P_{k+1|k} = \Phi_k P_{k|k} \Phi_k^T + (F_w \Delta t) Q (F_w \Delta t)^T$$

---

## 4. 点到平面观测模型与雅可比推导

当一帧去畸变后的激光点云传入时，任一点 ${}^L p_j$ 在世界系下的坐标为：
$${}^W p_j(x) = R \left( R_L^I \cdot {}^L p_j + p_L^I \right) + p$$

### 4.1 点面残差几何模型
在全局 ikd-Tree 中搜索 ${}^W p_j$ 的 5 个最近邻点，通过主成分分析（PCA）或奇异值分解（SVD）拟合出局部平面：
$$u_j^T q + d_j = 0$$
其中 $u_j \in \mathbb{R}^3$ 为单位法向量（$\|u_j\| = 1$），$d_j$ 为截距常数。
定义第 $j$ 个激光点的残差标量函数 $r_j(x)$：
$$r_j(x) = u_j^T \cdot {}^W p_j(x) + d_j$$
若点精确落在平面上，则 $r_j(x) = 0$。因此观测方程可写为：
$$0 = h_j(x) + v_j, \quad v_j \sim \mathcal{N}(0, \sigma^2)$$
其中 $h_j(x) = r_j(x)$，在源码中记录为：
```cpp
ekfom_data.h(i) = -norm_p.intensity; // norm_p.intensity 即 pd2 = u^T p + d
```

### 4.2 观测雅可比 $H_j = \left. \frac{\partial h_j(x \boxplus \delta x)}{\partial \delta x} \right|_{\delta x = 0}$ 解析推导

我们将 ${}^W p_j$ 展开扰动项：
$${}^W p_j(x \boxplus \delta x) = (R \mathrm{Exp}(\delta \theta)) \left[ (R_L^I \mathrm{Exp}(\delta \theta_{ext})) \cdot {}^L p_j + (p_L^I + \delta p_{ext}) \right] + (p + \delta p)$$

定义载体坐标系下的中间点：
$$p_I = R_L^I \cdot {}^L p_j + p_L^I$$
利用一阶近似 $\mathrm{Exp}(\delta \theta) \approx I + \delta \theta^\wedge$ 以及恒等式 $a^\wedge b = - b^\wedge a$：

1. **对位置误差 $\delta p$ 求导**：
   $$\frac{\partial h_j}{\partial \delta p} = u_j^T \cdot \frac{\partial {}^W p_j}{\partial \delta p} = u_j^T$$
2. **对姿态误差 $\delta \theta$ 求导**：
   $$\frac{\partial {}^W p_j}{\partial \delta \theta} = \frac{\partial}{\partial \delta \theta} \left( R (I + \delta \theta^\wedge) p_I \right) = - R p_I^\wedge$$
   代入残差：
   $$\frac{\partial h_j}{\partial \delta \theta} = u_j^T (- R p_I^\wedge) = (u_j^T R) (- p_I^\wedge) = (p_I^\wedge (R^T u_j))^T$$
3. **对外参平移 $\delta p_{ext}$ 求导**：
   $$\frac{\partial h_j}{\partial \delta p_{ext}} = u_j^T R = (R^T u_j)^T$$
4. **对外参旋转 $\delta \theta_{ext}$ 求导**：
   $$\frac{\partial h_j}{\partial \delta \theta_{ext}} = u_j^T R (- (R_L^I \cdot {}^L p_j)^\wedge) = (({}^L p_j)^\wedge (R_L^I)^T R^T u_j)^T$$

### 4.3 源码对照（[src/laserMapping.cpp#L777-L789](../src/laserMapping.cpp#L777-L789)）
在代码中，作者定义了中间变量 $C, A, B$：
```cpp
V3D C(s.rot.conjugate() * norm_vec);                     // C = R^T * u
V3D A(point_crossmat * C);                               // A = p_I ^ (R^T * u)
V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);// B = ({}^L p) ^ ((R_L^I)^T * C)

if (extrinsic_est_en) {
    // 雅可比前 12 列分别对应: delta_p(3), delta_theta(3), delta_theta_ext(3), delta_p_ext(3)
    ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, 
                                        VEC_FROM_ARRAY(A), 
                                        VEC_FROM_ARRAY(B), 
                                        VEC_FROM_ARRAY(C);
} else {
    ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, 
                                        VEC_FROM_ARRAY(A), 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}
```

---

## 5. 核心计算加速：矩阵求逆的维度反转

在标准卡尔曼滤波中，若有效特征点数为 $m \approx 2000$，观测向量维度为 $m$，状态维度 $n = 23$。

### 5.1 传统卡尔曼增益公式的困境
标准形式为：
$$K = P H^T (H P H^T + R)^{-1}$$
其中 $H P H^T + R$ 是一个 $m \times m$ 的稠密矩阵！
对一个 $2000 \times 2000$ 的矩阵求逆，计算复杂度为 $\mathcal{O}(m^3)$，即使使用最顶级的 CPU，耗时也需要数十毫秒，根本不可能实现实时性。

### 5.2 状态维度等价形式（Sherman-Morrison-Woodbury 恒等式）
利用矩阵求逆引理，卡尔曼增益可严格等价转换为：
$$K = (H^T R^{-1} H + P^{-1})^{-1} H^T R^{-1}$$
由于每个激光点的测距误差相互独立，观测协方差为对角阵 $R = \sigma^2 I_{m \times m}$。
因此：
$$H^T R^{-1} H = \frac{1}{\sigma^2} H^T H$$
$$K = \left( H^T H + \sigma^2 P^{-1} \right)^{-1} H^T$$

此时，需要求逆的矩阵为：
$$A = H^T H + \sigma^2 P^{-1} \in \mathbb{R}^{n \times n} = \mathbb{R}^{23 \times 23}$$

不仅如此，**由于只有前 12 个状态量（位姿与外参）存在观测雅可比，后 11 个状态（速度、零偏、重力）在 $H$ 中全为 0**！
$$H = \begin{bmatrix} H_{m \times 12} & 0_{m \times 11} \end{bmatrix}$$
$$H^T H = \begin{bmatrix} H_{12 \times m}^T H_{m \times 12} & 0_{12 \times 11} \\ 0_{11 \times 12} & 0_{11 \times 11} \end{bmatrix}$$
这意味着计算 $H^T H$ 仅仅需要对 $12 \times 12$ 大小的矩阵执行向量内积累加！

| 维度对比 | 传统形式 $H P H^T + R$ | FAST-LIO2 逆向形式 $H^T R^{-1} H + P^{-1}$ |
| :--- | :--- | :--- |
| **求逆矩阵尺寸** | $2000 \times 2000$ | **$23 \times 23$**（甚至分块化为 $12 \times 12$） |
| **求逆耗时** | $\approx 40 \sim 100 \text{ ms}$ | **$< 0.1 \text{ ms}$** |
| **算法整体帧率** | 无法突破 10 Hz | **稳定达到 50 ~ 100 Hz** |

源码实现位于 [include/IKFoM_toolkit/esekfom/esekfom.hpp#L1782-L1812](../include/IKFoM_toolkit/esekfom/esekfom.hpp#L1782-L1812)：
```cpp
cov P_temp = (P_/R).inverse();
Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_; 
P_temp.template block<12, 12>(0, 0) += HTH;
cov P_inv = P_temp.inverse(); // 仅对 23x23 矩阵求逆！

K_h = P_inv.template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h;
K_x.template block<n, 12>(0, 0) = P_inv.template block<n, 12>(0, 0) * HTH;
```

---

## 6. 流形上的高斯-牛顿迭代更新（IESKF）

由于点面距离关于位姿具有高度非线性，单次更新可能会线性化失真。FAST-LIO2 在流形切空间采用 Gauss-Newton 迭代：

在第 $k$ 轮迭代中：
1. 计算当前估计 $\hat{x}_k$ 下的残差 $h(\hat{x}_k)$ 和雅可比 $H_k$；
2. 考虑先验误差补偿项 $\hat{x}_k \boxminus \hat{x}_{prior}$；
3. 解出增量步长 $\delta x_k$：
   $$\delta x_k = - (H_k^T R^{-1} H_k + P^{-1})^{-1} \left( H_k^T R^{-1} h(\hat{x}_k) + P^{-1} (\hat{x}_k \boxminus \hat{x}_{prior}) \right)$$
4. 在流形上更新当前状态：
   $$\hat{x}_{k+1} = \hat{x}_k \boxplus \delta x_k$$
5. 检查每个维度的更新量是否小于预设容差阈值 $\epsilon$（`epsi` 数组，[src/laserMapping.cpp#L903](../src/laserMapping.cpp#L903)），若收敛或达到最大迭代次数（默认 3~4 次）则停止迭代。

迭代收敛后，将最新后验协方差矩阵赋值给滤波状态，进入下一阶段。
