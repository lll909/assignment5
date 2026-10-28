# RoboMaster Assignment5：MID360 离线建图与地图处理

本项目使用 Livox MID360 雷达及 IMU 的 rosbag，通过独立的 Point-LIO ROS 2 工作空间生成三维地图，再由 `mid360_mapping/map_processor_node` 处理完整地图并发布二维 `OccupancyGrid`。

实际环境：Ubuntu 24.04 LTS、ROS 2 Jazzy、C++17、PCL。Point-LIO 与 Livox 驱动分别在独立工作空间编译。

## 当前结果与验证边界

- 已在本机运行 Point-LIO，观察过配准点云与连续轨迹，并保存完整 PCD。
- 已从完整 PCD 完成体素降采样、高度筛选、二维投影、PCD/PGM/YAML 保存，以及 Nav2 加载历史二维地图。
- 本次展示的原始 PCD 发布日志为 **2,071,950 点**。点数随文件版本变化，以当前文件为准。
- 已在 RViz 的 `camera_init` 下同时显示完整三维点云与二维地图，两项均为 `Status: Ok`。
- 本次代码补充了显式有限值清理、可选统计离群点过滤、输入坐标系检查、保存错误检查及默认防覆盖保护。这些新增修改需要在 Ubuntu/Jazzy 本机编译运行；下方截图来自修改前已成功运行的版本，不能作为新增功能实测证据。

![三维地图与二维地图俯视叠加](docs/images/alignment_top.png)

![三维结构与二维投影斜视展示](docs/images/alignment_3d.png)

截图中全局 `No tf data` 提示仍存在；两个数据源与 Fixed Frame 相同，两项显示均正常。没有为消除提示添加未经验证的静态 TF。截图支持检查显示链路和主要轮廓对应关系，不证明里程计精度。

## 依赖与编译

现有工作空间：

| 路径 | 用途 |
|---|---|
| `~/assignment5` | 地图处理、显示配置和文档 |
| `~/ws_livox` | Livox ROS Driver2 与 CustomMsg |
| `~/ws_point_lio` | 实际 Point-LIO 源码与 `PCD/scans.pcd` |
| `~/ws_point_lio_ws` | Point-LIO colcon 工作空间 |

需要 ROS 2 Jazzy、`rclcpp`、`nav_msgs`、`sensor_msgs`、`std_srvs`、PCL、`pcl_conversions`、`pcl_ros`、RViz2、`python3-yaml` 和已编译的 `livox_ros_driver2`。Nav2 Map Server 仅用于可选的磁盘地图加载。

已有环境无需重新安装。若移到新电脑，可先安装功能包依赖：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
cd ~/assignment5
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select mid360_mapping
source install/setup.bash
```

Livox 驱动及 Point-LIO 不包含在本仓库中，须分别按各自所用 ROS 2 分支构建。迁移时应记录实际来源、分支/提交与配置；不能将 ROS 1 的同名仓库直接视为当前 ROS 2 版本。仓库旧模板 README 使用 Humble，本项目实际运行环境为 Jazzy。

## 最短复现：从已有 PCD 显示地图

此流程不需要 rosbag、Point-LIO 在线运行或 Map Server。先停止旧的 PCD 发布、地图处理和 RViz 进程，避免同名节点与重复发布。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/assignment5/install/setup.bash
cd ~/assignment5

ros2 launch mid360_mapping pcd_preview.launch.py \
  pcd_path:=/home/wzf/ws_point_lio/PCD/scans.pcd
```

新增入口同时启动 PCD 发布、地图处理和 RViz；使用实际时间，无需 `/clock`。默认创建带时间戳的 `maps_preview_日期_时间` 输出目录；**启动不会保存地图，已有地图默认不允许覆盖**。PCD 每 10 秒发布一次，大点云的首次显示与处理可能需要等待。

PCD 发布参数是已在本机确认的 `tf_frame`，不是 `frame_id`。本入口将它设为 `camera_init`。PCD 本身不携带 ROS TF；设置此标签的前提是文件坐标确实为 Point-LIO 的全局坐标，不能用修改标签代替几何变换。

话题：

| 话题 | 类型 | 坐标系/内容 |
|---|---|---|
| `/pcd_full_map` | `sensor_msgs/msg/PointCloud2` | `camera_init`，完整原始地图 |
| `/mapping/map_filtered` | `sensor_msgs/msg/PointCloud2` | `camera_init`，处理后障碍物点云 |
| `/mapping/map_2d` | `nav_msgs/msg/OccupancyGrid` | `camera_init`，二维障碍物投影 |

RViz 配置为 `src/mid360_mapping/rviz/pcd_alignment.rviz`。Fixed Frame 为 `camera_init`，Map Alpha 为 0.5。改变视角可检查俯视轮廓和立体结构；高度截断后的二维范围不必覆盖所有原始三维点。

## 处理流程与参数

```text
Point-LIO 完整 scans.pcd
→ pcl_ros 发布 /pcd_full_map
→ 检查输入坐标系
→ 去掉 XYZ 中含 NaN/Inf 的点
→ VoxelGrid 体素降采样
→ 可选 StatisticalOutlierRemoval
→ 按 z 高度筛选
→ 发布障碍物点云 + 投影为 OccupancyGrid
→ 手动触发保存服务
```

每次收到完整地图都独立处理，不把完整快照重复累加到旧地图。非有限 intensity 重置为 0；不参与几何清理判断。

| 节点参数 | 默认值 | 含义 |
|---|---|---|
| `input_topic` | `/pcd_full_map` | 完整点云输入 |
| `map_frame` | `camera_init` | 期望输入及输出坐标系；不一致时拒绝处理 |
| `output_directory` | `./maps` | 节点直接运行时的保存目录；新 launch 使用带时间戳目录 |
| `voxel_size` | `0.10` | 体素边长，米 |
| `grid_resolution` | `0.10` | 每个二维格子的边长，米 |
| `obstacle_height_min/max` | `0.15 / 1.50` | 节点直接运行默认高度，米 |
| `remove_outliers` | `false` | 是否启用统计离群点过滤 |
| `outlier_mean_k` | `20` | 邻域点数量 |
| `outlier_stddev` | `1.0` | 距离标准差倍数 |
| `overwrite_existing` | `false` | 保存时是否允许覆盖同名成果 |

**新展示 launch 的 `height_min/max` 默认是 `-10 / 10`**，用于复现已经成功的诊断展示，不代表导航障碍高度。它们对应节点的 `obstacle_height_min/max`。高度应相对于重力对齐后的地图理解，不能未经测量就声称是距地面高度。

栅格宽高为 `floor(跨度/分辨率)+1`，避免最外侧点落到边界外；原点为筛选后点云最小 x、y，方向为单位四元数。栅格总数上限为 2500 万，以避免异常范围造成过大分配。

占据格为 `100`，没有可靠观测的格子为 `-1`。目前没有射线清除，不主动生成空闲格 `0`。PGM 对应：障碍 `0`（黑）、未知 `205`（灰）、空闲 `254`（白）。PGM 保存时反转行顺序，使图片坐标与 OccupancyGrid 坐标对应。

### 参数对比

作业要求比较参数。已有不同历史点数来自不同文件版本，不能直接作为受控对比结果。应固定同一份 PCD、同一高度范围，每次只改变一个参数，并记录日志和截图。

例如比较基线和离群过滤，分别运行以下两次，停止前一次后再启动下一次：

```bash
# A：已验证展示参数的基线，另存到新目录
ros2 launch mid360_mapping pcd_preview.launch.py \
  output_directory:=$HOME/assignment5/maps_preview_A remove_outliers:=false

# B：只开启统计离群点过滤，大地图使用更长发布周期
ros2 launch mid360_mapping pcd_preview.launch.py \
  output_directory:=$HOME/assignment5/maps_preview_B remove_outliers:=true \
  publishing_period_ms:=60000
```

日志会报告原始点数、有限点数、体素后点数、离群过滤后点数（启用时）及高度过滤后点数。对比噪点是否减少、细柱与窄墙是否被误删。可进一步单独比较 `voxel_size:=0.20` 与 `0.10` 的细节/规模差异。**这些是待本机执行的实验，当前不填虚构数值，也不预先认定过滤更好。**

## 保存与地图文件验证

等待日志出现高度筛选后的非零点数和 RViz 二维地图后，在另一终端加载 ROS 环境，再手动调用：

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /mapping/save_artifacts std_srvs/srv/Trigger '{}'
```

保存目录以启动参数与服务响应为准，生成 `map.pcd`、`map.pgm`、`map.yaml`。其中 PCD 是**高度筛选后的障碍点云**，不是原始完整 Point-LIO 地图。完整地图保留在 `~/ws_point_lio/PCD/scans.pcd`。

保存服务会拒绝空缓存及已有同名文件，并检查 PCD 返回值和 PGM/YAML 文件流错误。三个文件依次写入，尚未实现三文件事务；若中途写失败，保留现场，使用新的空目录重试，不把残留文件视为成功成果。

```bash
# 将下面路径替换为实际输出目录
python3 ~/assignment5/scripts/validate_maps.py ~/assignment5/maps_preview_A
```

脚本检查 PCD 字段/点数和普通 binary 数据长度、PGM 宽高与像素长度、YAML 图像引用/分辨率/原点/阈值，并报告像素统计。它不检查几何精度；ASCII/compressed PCD 仅检查头部，不完整解码。当前打包审查未包含真实地图二进制，尚不能声称它们通过此脚本。

最终选定并验证地图后，放入仓库根目录的 `maps/`：

```text
maps/
├── map.pcd
├── map.pgm
└── map.yaml
```

可以用安全复制脚本整理选定结果，先校验源文件，再复制；目标已有地图时会拒绝覆盖：

```bash
python3 ~/assignment5/scripts/finalize_maps.py \
  ~/assignment5/maps_preview_A ~/assignment5/maps
```

修改后的 `.gitignore` 明确保留以上三个最终文件，排除历史地图目录、rosbag 和编译产物。

可选：Nav2 Map Server 加载 YAML 后通常发布 `/map`，其 frame 可与原处理节点不同。最终截图优先直接订阅 `/mapping/map_2d`，无需为展示额外制造 TF。

## 需要重新建图时：Point-LIO 与 rosbag

已有 PCD 的展示不运行这一节。确需重新建图时，先备份原始 PCD，避免 Point-LIO 的保存机制覆盖原成果。

终端 1，沿用本机已成功使用的入口：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_point_lio_ws/install/setup.bash
ros2 launch point_lio point_lio.launch.py
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
ros2 bag play ~/bags/rosbag2_2026_05_28-17_11_13 \
  --clock --rate 0.5 --start-paused \
  --topics /livox/lidar_192_168_1_140 /livox/imu_192_168_1_140
```

确认 Point-LIO 的实际配置使用上述雷达/IMU 话题及 CustomMsg；使用模拟时间的节点应启用 `use_sim_time`。时间同步、雷达类型、外参和保存开关取决于本机所用 Point-LIO 分支配置，**此补丁未包含该外部工作空间，无法独立验证这些配置或它的保存触发方式**。节点就绪后按空格开始播放。

历史运行输出包括 `/cloud_registered`、`/path` 和 `/Laser_map`。`/path` 在重启后不会从 PCD 自动恢复。Point-LIO 的完整地图保存成功后再进入 PCD 处理流程，不能将未经确认的 `/Laser_map` 小快照当成完整地图。

`mapping_node.cpp` 是保留的模板输入节点：只转发当前帧并统计 IMU，未实现 LIO、配准或轨迹；它不能替代 Point-LIO，原有 `mapping.launch.py` / `replay.launch.py` 也不自动集成外部 Point-LIO。因此本项目的真实建图与地图处理采用两个独立步骤。该模板节点没有地图积累时，保存服务明确返回失败。

## 限制

rosbag 实际场景未知，不能仅凭环形外观判定地图好坏。当前未做真实场景精度评估、闭环验证、基于射线的空闲空间推断或自主导航测试；本输出是点云投影障碍物栅格图。高度范围和离群过滤强度需要结合真实数据选择。


