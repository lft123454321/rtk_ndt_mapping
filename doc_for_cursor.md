## 输入接口说明
### 激光点云
- 话题名: /os_cloud_node/points
- 消息类型: sensor_msgs/PointCloud2
- 频率: 10Hz
- 安装方向: 
  - X轴与载具前进方向相反
  - Y轴为X轴逆时针旋转90度（俯视状态下）
  - Z轴朝上

### 独立IMU
- 话题名: /imu/data
- 消息类型: sensor_msgs/Imu
- 频率: 100Hz
- 传感器型号: 3DM-GX5-25
- 数据来源: roslaunch microstrain_inertial_driver microstrain.launch
- 安装方向: 
  - X轴与激光雷达一致
  - Y轴与激光雷达相反
  - Z轴与激光雷达相反

### 组合导航
- 数据来源: RTK+IMU组合导航
- 安装方向: 
  - X轴与激光雷达Y轴一致
  - Y轴与激光雷达X轴相反
  - Z轴与激光雷达Z轴一致

#### IMU话题
- 话题名: /imu2
- 消息类型: sensor_msgs/Imu
- 频率: 100Hz
- 加速度: m/s^2
- 角速度: rad/s
- roll: orientation_covariance[0]，单位deg
- pitch: orientation_covariance[1]，单位deg
- yaw: orientation_covariance[2]，单位deg，方向: 向北为0，顺时针增加（TBD），RTK非固定解时不可用

#### GNSS话题
- 话题名: /gnss
- 消息类型: sensor_msgs/NavSatFix
- 频率: 100Hz
- status.status: 3为固定解，其他为非固定解

### 保存地图服务
- 作用：需要保存地图时调用一次该服务，即可将地图保存在本地

## 输出接口说明

### 建图结果点云
- 话题名: /cloud_map
- 消息类型: sensor_msgs/PointCloud2
- 发布频率：10s

## 运行逻辑说明

- 第一次出现固定解的时候开始建图，以第一次出现固定解的位置（经纬高转换到ENU坐标系）和航向（imu2中的orientation_covariance[2]）确定本地坐标系原点和方向，取gnss的altitude为z=0，x方向与东方向对齐，y方向与北方向对齐
- 在RTK固定解的时候，强制使用RTK位姿（经纬高转换到ENU坐标系得到xyz，与建图初始位置相减得到本地坐标系坐标，姿态直接使用imu2的roll pitch yaw）来将实时点云拼合到点云地图中
- 在RTK无固定解的时候，使用ndt_mapping.cpp方法进行激光NDT建图
- 定期发布建图结果
- 保存地图服务被调用的时候，将地图保存为pcd文件