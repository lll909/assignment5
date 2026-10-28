# Point-LIO ROS 2 实际版本与本机修改

本目录记录 Assignment5 使用的外部建图后端，不包含 Point-LIO 的完整源码、rosbag 或 PCD。既有电脑无需重新下载、编译或回放即可继续展示现有地图。

## 来源与版本

- 仓库：[Thetrafficlights/Point-LIO-ros2](https://github.com/Thetrafficlights/Point-LIO-ros2)
- 本机 HEAD：`155f46da0b1eb3fc7c7facf43b00cb124631c807`
- 本机修改：`CMakeLists.txt`、`config/mid360.yaml`；未跟踪的 `PCD/` 是生成数据。
- 原始构建文件 Git blob：`12044e4277970e0b6311d74dc98208c2de1d6fc7`
- 按本机差异还原后的 Git blob：`015e31f19aeedd84ac88ce92fa31de31af2b1633`，与用户 diff 中的新文件前缀 `015e31f` 一致。

本目录的 `config/mid360.yaml` 与 `launch/point_lio.launch.py` 是用户上传的实际文件。`CMakeLists.local.txt` 是按用户 diff 准确还原的构建文件快照；不命名为 CMakeLists.txt，以避免被 colcon 当成独立构建项目。实际修改使用 `cxx17.patch`。

## C++17 修改

删掉重复 `ADD_COMPILE_OPTIONS(-std=c++14)`、CXX_FLAGS 中的旧标准选项及 `-std=c++0x`；保留 `-O3`、`-pthread`、`-fexceptions`，由 `CMAKE_CXX_STANDARD 17` 统一设置 C++ 标准。

这是本机已使用的构建修改记录，不表示原仓库在其他平台也必须这样修改。补丁已在指定版本的原始 CMake 文件上执行 `git apply --check` 和结果哈希核验；审查环境没有重新编译 ROS 2/Point-LIO。

## 配置要点

| 参数 | 上传配置中的值 |
|---|---|
| `common.lid_topic` | `livox/lidar_192_168_1_140` |
| `common.imu_topic` | `livox/imu_192_168_1_140` |
| `preprocess.lidar_type` | `1`（此实现的 Livox 分支） |
| `mapping.imu_en` | `true` |
| `use_imu_as_input` | `false`（算法模式选择，不等于关闭 IMU） |
| `filter_size_surf` / `filter_size_map` | `0.1` / `0.1` |
| `mapping.extrinsic_est_en` | `false` |
| `mapping.extrinsic_T` | `[-0.011, -0.02329, 0.04412]` |
| `mapping.extrinsic_R` | 单位矩阵 |
| `publish.path_en` / `scan_publish_en` / `tf_send_en` | `true` / `true` / `true` |
| `pcd_save.pcd_save_en` / `interval` | `true` / `-1` |

没有 namespace 时，这两个相对输入名对应 rosbag 的绝对话题。使用 namespace 时需要明确处理名称解析。`acc_norm`、IMU 饱和值及外参均按既有运行配置保存，本次交付未重新标定，也未根据 rosbag 重新验证单位；不能把模板参数当成标定精度证明。

## 既有本机启动

以下使用实际 launch 支持的 `point_lio_cfg_dir` 参数，避免无意读取其他版本的安装配置。先备份原有 `~/ws_point_lio/PCD/scans.pcd`；只有需要重新建图时才运行。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_point_lio_ws/install/setup.bash

ros2 launch point_lio point_lio.launch.py \
  point_lio_cfg_dir:=$HOME/assignment5/docs/point_lio/config/mid360.yaml \
  rviz:=False
```

`rviz:=False` 只关闭它自带的可视化，不关闭建图。上传的 launch 将 RViz 配置路径和 `.rviz` 拆成两个命令行参数，因此这里单独关闭其 RViz，不依赖该参数组合。需要在线查看轨迹时，另开 RViz，Fixed Frame 设为 `camera_init`，显示 `/cloud_registered`、`/path`；已保存地图的展示则使用 Assignment5 的 `pcd_preview.launch.py`。

上游指定提交中的节点名为 `/laserMapping`，位姿消息话题为 `/aft_mapped_to_init`，轨迹为 `/path`，配准点云为 `/cloud_registered`，另有 `/Laser_map`。发布者存在不代表数据完整；完整地图处理使用退出后保存的 PCD。

另一个终端回放：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash

ros2 bag play ~/bags/rosbag2_2026_05_28-17_11_13 \
  --clock --rate 0.5 --start-paused \
  --topics /livox/lidar_192_168_1_140 /livox/imu_192_168_1_140
```

确认节点准备好后按空格开始。ROS 节点若使用模拟时间，应在播放前显式启用并核对 `use_sim_time`，不要在运行中混用不同的时间基准。本次没有因文档整理而重放 rosbag。

指定提交的 `src/laserMapping.cpp` 在主处理循环结束后，若缓存非空且 `pcd_save_en` 启用，将点云写入编译时 `ROOT_DIR + PCD/scans.pcd`。`ROOT_DIR` 来自源码目录；正常停止程序后应检查保存日志、文件时间和 PCD 头部，不能仅凭退出推断保存成功。该过程可能覆盖同名原始地图，重新建图前须自行备份。

## 在新目录还原构建

下面是为另一台已安装 ROS 2 Jazzy 和 Livox 驱动的电脑提供的还原流程；当前电脑已编译成功，不需要再执行。使用新的时间戳目录，避免覆盖既有工程。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash

repro_root="$HOME/point_lio_repro_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$repro_root/ws/src"

git clone https://github.com/Thetrafficlights/Point-LIO-ros2.git "$repro_root/point_lio"
git -C "$repro_root/point_lio" checkout --detach 155f46da0b1eb3fc7c7facf43b00cb124631c807

git -C "$repro_root/point_lio" apply --check "$HOME/assignment5/docs/point_lio/cxx17.patch"
git -C "$repro_root/point_lio" apply "$HOME/assignment5/docs/point_lio/cxx17.patch"
cp "$HOME/assignment5/docs/point_lio/config/mid360.yaml" "$repro_root/point_lio/config/mid360.yaml"
mkdir -p "$repro_root/point_lio/PCD"

ln -s "$repro_root/point_lio" "$repro_root/ws/src/point_lio"
cd "$repro_root/ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

新的完整地图会写到新源码目录的 `PCD/scans.pcd`，不是原有 `~/ws_point_lio/PCD/scans.pcd`。可用 `git hash-object "$repro_root/point_lio/CMakeLists.txt"` 对照上面的本机文件 blob。

第三方算法及配置来源归属于上述仓库；本项目没有声称自行实现 Point-LIO。复现材料改善了版本和参数可追溯性，不代表跨电脑已完成端到端验证。
