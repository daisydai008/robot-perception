# 相机与标定配置

本目录用版本控制保存相机、标定板和 DDS 配置，以及已签收的相机外参。它们不随 package 分发，程序也不会自动加载，使用时须显式给出路径。凭据不放进仓库；本目录含内网地址和设备序列号，创建远端或公开发布前须复核。

| 路径 | 状态 | 说明 |
| --- | --- | --- |
| [boards/dfoptix_q12-290-20.json](boards/dfoptix_q12-290-20.json) | 当前 | DFOPTIX Q12-290-20 普通棋盘：12×9 方格、11×8 内角点，单格 20 mm 经用户确认（量具和不确定度未记录）；黑白格区域 240×180 mm，整板 290×230 mm，厚 3.2 mm。取代旧记录 `Q12-12x9`（已删除，hash 见 provenance.json），数值不变。 |
| [cameras/d435/fastdds_workstation.xml](cameras/d435/fastdds_workstation.xml) | 当前，使用前复核 | workstation 接收 Jetson D435 话题的 Fast DDS profile。2026-11-09 在 workstation 6.6.7.94 与 Jetson 6.6.7.100 之间验证收流；2026-11-10 的标定也用它接收。地址或网卡变化时先更新再用。 |
| [cameras/d435/extrinsics_base_link_20261110.json](cameras/d435/extrinsics_base_link_20261110.json) | 当前，用户签收，限定范围 | 头部 D435 外参：`base_link` ← `d435_color_optical_frame`，另附 `base_link` ← `d435_link`。由厂商 URDF、Intel D435 机械模型和驱动出厂 TF 拼接而成，经两个静态姿态的双臂叠图和原生深度核对（修正量 4.3 mm / 0.58° 和 5.1 mm / 0.53°，均未采用）。仅适用于头部固定、静态使用，没有验证时间同步。方法见[头部 D435 相机标定](../docs/calibration/d435_head.md)。 |

收流时两端使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=0`。在 workstation 上使用上述 profile：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOT_PERCEPTION_REPO/config/cameras/d435/fastdds_workstation.xml"
```

Jetson 端的驱动与 DDS 配置保存在 Jetson 部署目录中，本目录没有复制。REGRIND 旧相机启动链的退役配置已按用户要求删除。迁移、删除记录和逐文件 hash 见 [provenance.json](../provenance.json) 的 `post_migration_changes`。

## D435 彩色流：本仓库统一 848×480@30

2026-11-10 起，按用户决定，本仓库的标定和感知统一使用 D435（SN 348122072721）彩色 848×480@30 rgb8。Jetson 部署配置不改，仍为 640×480@30。使用前在 workstation 上切换正在运行的驱动：

```bash
ros2 param set /cameras/d435 rgb_camera.color_profile 848x480x30
ros2 param set /cameras/d435 enable_color false
ros2 param set /cameras/d435 enable_color true
```

切换会影响这台相机的所有订阅者，切换前须确认 teleop 或数据采集没有在用。驱动重启后会恢复部署默认的 640×480。深度保持 640×480，align 关闭。

实测 CameraInfo 为 plumb_bob、D 全 0，fx=603.646、fy=603.362、cx=430.930、cy=245.467，视野约 70.2°×43.4°，帧率 30.0 Hz。按厂家内参，640×480 模式只是把 cx 平移 104 px，等价于 848×480 的中间裁切。因此外参（相机位姿）对两种分辨率通用；内参和画面内的像素坐标须按实际分辨率取用。
