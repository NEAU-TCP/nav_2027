# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml/badge.svg?branch=main)](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## 本项目的点云时间窗口

仿真参数集中在 `pb2025_nav_bringup/config/simulation/nav2_params.yaml`：

```text
Small Point-LIO（逐点估计和运动补偿）
  → cloud_registered_full（0.1 秒短批次，camera_init）
  → loam_interface → registered_scan（odom）
                       ├─ ROGMap + lidar_odometry → 占据地图、ESDF
                       └─ small_gicp：最近 1 秒滑动窗口 + 先验 PCD → map→odom
```

- `small_point_lio.cloud_batch_period`：输出短批次的目标时长，默认 0.1 秒。
  按处理后的数据时间触发发布；实际批次长度受输入分段粒度影响。
  不改变逐点估计和里程计更新，也不等待 GICP 的长窗口。
- `small_gicp_relocalization.cloud_window_duration`：GICP 保留的历史时长，默认 1.0 秒。
  根据点云消息时间戳淘汰旧批次，每 0.5 秒对窗口快照做一次合并、体素降采样和配准。
  首次输入不足一个完整窗口也可配准；没有新数据时不重复配准。
- `small_gicp_relocalization.max_window_points`：窗口原始点数上限，默认 600000。
  超限时淘汰最旧批次；单批超过上限时跳过并报警。实际保留时长可能因此缩短。

窗口只接受已运动补偿、位于固定 `odom_frame` 下的注册点云，不能直接拼接雷达局部坐标系点云。
空点云、非有限坐标和重复时间戳不会增加配准权重。时间戳回退会清空窗口并重置配准修正，
防止仿真重启前后的数据混合。合批不改变现有 TF 树。

启动日志输出窗口配置；运行日志中的 `GICP window: frames=... span=... input=... downsampled=...`
可用于确认多批次确实进入配准。`span` 是最早和最新批次结束时间之差，不包含首批自身的扫描时长。

底盘 TF 由 `sensor_scan_generation` 根据里程计独立发布（最高 100 Hz），不再等待点云与逐点里程计同步。
`sensor_scan` 使用点云时间戳查询 TF；查不到时保留最新批次，待后续里程计到达后重试。
外参不可用时等待有效 TF，不以单位变换代替。`fake_vel_transform` 在首次收到里程计之前以零角度发布有效 TF，
控制器超时判断使用节点时钟，兼容仿真时间。

## 匹配结果检查

优化器返回“收敛”仅表示局部优化停止，不代表找到正确位置。每次配准额外计算源点变换后到先验地图的几何近邻距离，
使用仿真 YAML 中的以下参数检查结果：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `min_overlap` | 0.4 | 至少 40% 的源点在 `2 * max_rmse` 半径内有地图近邻 |
| `max_rmse` | 0.2 m | 有近邻支持的源点，其均方根距离上限 |
| `max_translation_delta` | 1.0 m | 相对上次接受的 `map→odom` 校正的平移变化上限 |
| `max_rotation_delta` | 0.35 rad | 校正的旋转变化上限 |

首次配准相对于 `init_pose` 检查校正变化。检查未通过时保留上次有效 TF；首次尚未成功时不发布未经验证的校正。
初始位置与 `init_pose` 相差较大时，先通过 RViz 的 `2D Pose Estimate` 提供接近实际位置的初值。
这些检查可以拦截低重叠、高残差和突跳，无法保证区分几何重复区域中的所有错误匹配。

日志 `GICP accepted` / `GICP rejected` 输出重叠比例、RMSE、校正变化量和拒绝原因。
校正变化限制作用于定位修正，不限制机器人的移动速度。
`max_iterations` 控制单次局部优化的迭代上限，当前为 30；达到上限仍未收敛时继续拒绝结果。

仿真输入 `ign_sim_pointcloud_tool` 保留 Gazebo 快照的采集时间，所有点的相对 `time` 为零。
不应给 GPU lidar 快照填入 `0～0.1 s` 的逐点扫描时间；Small Point-LIO 会按这些时间处理运动，
使运动时的注册点云发生形变。`point_time_scale: 1.0` 保持秒单位，实物雷达的真实逐点时间不受此转换器影响。

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the Point-LIO `camera_init` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```
