# RTK-NDT Mapping Package

## 功能简介

本包实现了基于RTK与NDT的激光建图，支持RTK固定解锚定、NDT激光配准、地图分块管理、分块降采样、分块保存、离线bag解析等功能。

## 主要特性

- **RTK固定解锚定**：RTK固定解时，直接用GNSS+IMU2姿态锚定位姿。
- **NDT建图**：RTK不可用时自动切换为NDT激光建图。
- **地图分块管理**：地图以100x100m（可配置）为单位分块，便于大规模地图管理与存储。
- **NDT局部拼合**：NDT匹配时只拼合当前位姿附近50m（可配置）内的分块作为target map，提升效率。
- **可视化降采样**：发布地图时，各分块先降采样到0.5m（可配置），再拼合发布，提升可视化效率。
- **分块保存**：保存地图时，各分块单独降采样（0.1m，可配置）并保存为block_x_y.pcd。
- **离线bag解析**：支持直接读取bag文件顺序解析消息，避免实时订阅与播包速度影响，适合大包调试。
- **多种轨迹发布**：支持发布RTK轨迹、NDT轨迹、混合轨迹，便于对比分析。

## 主要参数（config/rtk_ndt_mapping.yaml）

- `ndt_resolution`：NDT体素分辨率
- `ndt_search_radius`：NDT target map拼合半径（单位m）
- `map_block_size`：地图分块尺寸（单位m）
- `vis_downsample_res`：可视化发布降采样分辨率
- `block_downsample_res`：分块保存降采样分辨率
- `offline_bag_mode`：是否启用离线bag解析
- `offline_bag_path`：离线bag文件路径
- 其他常规参数见yaml文件

## 主要话题

- `/cloud_map`：可视化降采样后的全局地图
- `/rtk_pose_array`：RTK轨迹（PoseArray）
- `/hybrid_pose_array`：混合轨迹（PoseArray）
- `/ndt_pose_array`：NDT轨迹（PoseArray）

## 主要服务

- `/save_map`：保存分块地图（支持分辨率、路径参数）
- `/pub_map`：立即发布一次地图

## 离线bag解析模式

- 通过`offline_bag_mode`和`offline_bag_path`参数控制。
- 直接顺序读取bag文件，调用各回调，发布与在线模式一致。
- 控制台实时显示bag总时长与已解析进度。

## 地图分块与保存

- 地图以`map_block_size`为单位分块，所有点云按分块存储。
- NDT匹配和可视化、保存均按分块拼合、降采样。
- 保存时每个分块单独保存为`block_x_y.pcd`。

## 使用方法

1. 配置参数（config/rtk_ndt_mapping.yaml）
2. 启动节点
   ```bash
   roslaunch rtk_ndt_mapping rtk_ndt_mapping.launch
   ```
3. 离线bag解析
   - 设置`offline_bag_mode: true`和`offline_bag_path`，直接运行主节点即可自动离线解析。
4. 保存地图
   ```bash
   rosservice call /save_map "resolution: 0.1 destination: '/your/save/dir'"
   ```

## 依赖
- ROS Noetic
- PCL 1.10+
- geodesy
- rosbag
- message_generation

## 典型应用场景
- 大规模激光建图
- RTK+激光融合定位
- 离线大包高效建图与可视化
