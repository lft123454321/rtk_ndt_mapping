# RTK-NDT Mapping Package (English)

[中文版文档请点击这里](README_cn.md)

## Overview

This package implements RTK+NDT-based LiDAR mapping, supporting RTK anchor, NDT scan matching, map tiling, per-tile downsampling, per-tile saving, and offline bag parsing.

## Main Features

- **RTK Anchor**: When RTK is fixed, use GNSS+IMU2 pose directly for mapping.
- **NDT Mapping**: Automatically switch to NDT scan matching when RTK is not available.
- **Map Tiling**: The map is divided into blocks (default 100x100m, configurable) for efficient large-scale management and storage.
- **Local NDT Target**: Only blocks within a configurable radius (default 50m) of the current pose are merged as the NDT target map, improving efficiency.
- **Visualization Downsampling**: Each block is downsampled (default 0.5m, configurable) before merging and publishing for visualization.
- **Per-tile Saving**: When saving, each block is downsampled (default 0.1m, configurable) and saved as block_x_y.pcd.
- **Offline Bag Parsing**: Supports direct sequential parsing of bag files, avoiding real-time subscription and playback speed issues, suitable for large bag debugging.
- **Multiple Trajectory Publishing**: Publishes RTK, NDT, and hybrid trajectories for comparison and analysis.

## Key Parameters (see config/rtk_ndt_mapping.yaml)

- `ndt_resolution`: NDT voxel resolution
- `ndt_search_radius`: NDT target map merge radius (meters)
- `map_block_size`: Map block size (meters)
- `vis_downsample_res`: Visualization downsampling resolution
- `block_downsample_res`: Per-block saving downsampling resolution
- `offline_bag_mode`: Enable offline bag parsing
- `offline_bag_path`: Path to offline bag file
- Other standard parameters, see yaml file

## Main Topics

- `/cloud_map`: Downsampled global map for visualization
- `/rtk_pose_array`: RTK trajectory (PoseArray)
- `/hybrid_pose_array`: Hybrid trajectory (PoseArray)
- `/ndt_pose_array`: NDT trajectory (PoseArray)

## Main Services

- `/save_map`: Save tiled map (supports resolution and path arguments)
- `/pub_map`: Publish map immediately

## Offline Bag Parsing Mode

- Controlled by `offline_bag_mode` and `offline_bag_path` parameters.
- Reads bag file sequentially, calls callbacks, and publishes as in online mode.
- Console displays total and parsed bag duration in real time.

## Map Tiling and Saving

- The map is divided into blocks of `map_block_size`, all points are stored per block.
- NDT matching, visualization, and saving are all performed by merging and downsampling blocks.
- Each block is saved as `block_x_y.pcd`.

## Usage

1. Configure parameters in `config/rtk_ndt_mapping.yaml`
2. Launch the node:
   ```bash
   roslaunch rtk_ndt_mapping rtk_ndt_mapping.launch
   ```
3. For offline bag parsing:
   - Set `offline_bag_mode: true` and specify `offline_bag_path`, then run the main node.
4. Save the map:
   ```bash
   rosservice call /save_map "resolution: 0.1 destination: '/your/save/dir'"
   ```

## Dependencies
- ROS Noetic
- PCL 1.10+
- geodesy
- rosbag
- message_generation

## Typical Applications
- Large-scale LiDAR mapping
- RTK+LiDAR fusion localization
- Efficient mapping and visualization from large offline bags
