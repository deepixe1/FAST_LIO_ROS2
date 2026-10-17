# 04. 激光雷达点云预处理模块

在多传感器融合 SLAM 中，不同厂商的激光雷达具有完全不同的通信协议、数据结构与时间戳格式。FAST-LIO2 的预处理模块（[src/preprocess.h](../src/preprocess.h) 与 [src/preprocess.cpp](../src/preprocess.cpp)）承担了“将异构点云统一抽象为算法核心标准格式”的关键职责。

本章节深度解析：多雷达协议适配机制、“`curvature` 充当相对时间戳”的核心工程技巧、点云过滤规则，以及如何接入一款自研/新型激光雷达。

---

## 1. 核心设计：利用 `curvature` 字段传递微秒级时间戳

在激光雷达扫描去畸变时，算法必须知道每一个激光反射点的精确发射或接收时刻。然而，PCL 官方库的标准点类型（如 `pcl::PointXYZI`）并没有包含逐点时间戳字段。

为了避免自定义包含特殊时间字段的 PCL 点类型所带来的序列化性能损耗与编译复杂度，FAST-LIO2 巧妙地**复用了标准类型 `pcl::PointXYZINormal` 中的浮点字段 `curvature`（曲率）来存储该点相对于当前扫描帧起始时刻的时间偏移量**：

$$\mathrm{point.curvature} \equiv \Delta t_i = t_i - t_{\mathrm{start}} \quad (\text{单位：毫秒 ms})$$

### 1.1 关键流转路径跟踪
1. **预处理解析时刻**（[src/preprocess.cpp#L180-L181](../src/preprocess.cpp#L180-L181)）：
   ```cpp
   // 将纳秒 offset_time 换算为毫秒，存入 curvature
   pl_full[i].curvature = msg->points[i].offset_time / float(1000000);
   ```
2. **计算扫描结束时间时刻**（[src/laserMapping.cpp#L406](../src/laserMapping.cpp#L406)）：
   ```cpp
   // 取最后一个点的 curvature（毫秒转秒）作为整帧雷达扫描跨度
   lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
   ```
3. **IMU 运动补偿去畸变时刻**（[src/IMU_Processing.hpp#L315-L317](../src/IMU_Processing.hpp#L315-L317)）：
   ```cpp
   // 将点的时间除以 1000 转换为秒，与 IMU 轨迹的时间戳进行插值匹配
   for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --) {
       dt = it_pcl->curvature / double(1000) - head->offset_time;
       ...
   }
   ```

> [!WARNING]
> **绝对禁忌**：
> 如果你在下游代码中误以为 `point.curvature` 是几何表面曲率并对其进行计算或清零，整个 FAST-LIO2 的去畸变与时间同步逻辑将彻底崩溃！

---

## 2. 支持的激光雷达类型与协议适配

系统通过 `preprocess.lidar_type` 枚举参数（[src/preprocess.h#L14-L20](../src/preprocess.h#L14-L20)）支持多种主流传感器：

| 雷达类型枚举 | 代表型号 | 接收 ROS 2 消息类型 | 时间戳原始格式 |
| :--- | :--- | :--- | :--- |
| `AVIA` (1) | Livox Avia, Horizon | `livox_ros_driver2::msg::CustomMsg` | 点级 `offset_time`（单位：纳秒 ns） |
| `VELO16` (2) | Velodyne VLP-16, 32C, 64 | `sensor_msgs::msg::PointCloud2` | 自定义点结构中的 `time` 字段（秒） |
| `OUST64` (3) | Ouster OS1-16, 32, 64, 128 | `sensor_msgs::msg::PointCloud2` | 点级 `t` 字段（纳秒） |
| `MID360` (4) | Livox Mid-360 | `sensor_msgs::msg::PointCloud2` | 专有点结构中解析的时间字段 |

### 2.1 Livox Avia / Horizon 处理流程 (`avia_handler`)
Livox 雷达采用非重复扫描机制，使用专用消息 `CustomMsg`：
- **Tag 过滤**：Livox 驱动中带有回波反射质量标志位 `tag`。代码中检查 `(tag & 0x30) == 0x10 || (tag & 0x30) == 0x00`，滤除雨雾、飞虫或玻璃等低可信度噪点；
- **重复点抑制**：相邻点若三维欧氏坐标差值小于 $10^{-7}\text{ m}$，视为硬件重采样冗余点并剔除；
- **盲区剔除**：若点到原点的平方距离小于 `blind * blind`，予以丢弃。

### 2.2 机械旋转式雷达（Velodyne / Ouster）处理流程
机械雷达以恒定转速旋转，每圈输出点带有线号 `ring` 与测量时间。
- **Ouster 结构体注册**（[src/preprocess.h#L102-L113](../src/preprocess.h#L102-L113)）：
  通过 PCL 的 `POINT_CLOUD_REGISTER_POINT_STRUCT` 宏将 Ouster 的 `(x, y, z, intensity, t, reflectivity, ring, ambient, range)` 结构体反射给 PCL；
- **时间单位转换尺度 `time_unit_scale`**：
  根据配置文件中指定的 `preprocess.timestamp_unit`（秒、毫秒、微秒、纳秒），统一乘以缩放系数折算为毫秒保存到 `curvature`。
- **无时间戳降级策略**：
  若老式雷达驱动未输出逐点时间戳（`points.back().time <= 0`），代码通过方位角差值计算当前点所在的旋转角度，并假定旋转角速度恒定（`0.361 * SCAN_RATE`），强制估算近似时间偏移。

---

## 3. 直接法（Raw Points）vs 特征提取（Feature Extraction）

在传统 LOAM 类方法中，必须先提取线特征（Edge）和面特征（Surf）。然而在 FAST-LIO2 中，**默认配置是关闭特征提取的**（`feature_extract_enable: false`）。

```
点云进入预处理:
   |
   +--> feature_extract_enable == true  --> 计算曲率与跃变 --> 提取角点和面点 (较慢)
   |
   +--> feature_extract_enable == false --> 仅做空间下采样与盲区过滤 --> 直接输出全部点 (极快，推荐)
```

### 为什么 FAST-LIO2 可以抛弃特征提取？
1. **ikd-Tree 的超高效率**：传统方法提取特征是为了减少点数以迎合低效的 KD-Tree。而 FAST-LIO2 的 ikd-Tree 支持每秒数十万点的查询与更新，直接使用密集体素点云完全不会产生算力瓶颈；
2. **更高鲁棒性**：在隧道、森林、旷野等非结构化环境中，规则的线/面特征极其稀疏，基于硬编码阈值的特征提取往往彻底失效。直接法充分保留了环境中的微小起伏几何信息。

---

## 4. 实战：如何接入自研或新型雷达（以禾赛速腾为例）

若需要接入 Hesai（禾赛）或 Robosense（速腾）机械雷达，按照以下步骤扩展即可：

### 第一步：在 `src/preprocess.h` 中注册点结构
在命名空间中定义该雷达的点格式并向 PCL 注册：
```cpp
namespace hesai_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)
```

### 第二步：在 `src/preprocess.cpp` 中编写专用 Handler
仿照 `velodyne_handler` 编写 `hesai_handler`：
```cpp
void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
    pl_surf.clear();
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    double start_time = pl_orig.points[0].timestamp;

    for (int i = 0; i < plsize; i++) {
        if (i % point_filter_num != 0) continue;

        double dist = pl_orig.points[i].x * pl_orig.points[i].x + 
                      pl_orig.points[i].y * pl_orig.points[i].y + 
                      pl_orig.points[i].z * pl_orig.points[i].z;
        if (dist < blind * blind) continue;

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        // 关键：时间偏移转换为毫秒 (ms)
        added_pt.curvature = (pl_orig.points[i].timestamp - start_time) * 1000.0;

        pl_surf.push_back(added_pt);
    }
}
```

### 第三步：在 `process` 分发函数中添加分支
在 `Preprocess::process` 函数的 `switch(lidar_type)` 中调用新增的 `hesai_handler`，并在 YAML 文件中指定相应的类型编号即可。
