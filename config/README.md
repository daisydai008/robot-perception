# 相机与标定配置

本目录用版本控制保存相机、标定板和 DDS 配置。原始图像、session 和诊断记录放在本仓库的 `data/vision_calibration/`，只保存在本机，不入 git。两者都不随 package 分发，程序也不会自动加载，使用时须显式给出路径。凭据不放进仓库；本目录含内网地址和设备序列号，创建远端或公开发布前须复核。

| 路径 | 状态 | 说明 |
| --- | --- | --- |
| [boards/dfoptix_q12-290-20.json](boards/dfoptix_q12-290-20.json) | 当前 | DFOPTIX Q12-290-20 普通棋盘：12×9 方格、11×8 内角点，单格 20 mm 经用户确认（量具和不确定度未记录）；黑白格区域 240×180 mm，整板 290×230 mm，厚 3.2 mm。取代旧记录 `Q12-12x9`（已删除，hash 见 provenance.json），数值不变。 |
| [cameras/d435/fastdds_workstation.xml](cameras/d435/fastdds_workstation.xml) | 当前，使用前复核 | workstation 接收 Jetson D435 话题的 Fast DDS profile。2026-11-09 在 workstation 6.6.7.94 与 Jetson 6.6.7.100 之间验证收流，与记录 `data/vision_calibration/d435_dds_20261109_200102/dds_workstation.xml` 逐字节相同。地址或网卡变化时先更新再用。 |

2026-11-09 验证收流时，两端使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=0`。在 workstation 上使用上述 profile：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOT_PERCEPTION_REPO/config/cameras/d435/fastdds_workstation.xml"
```

Jetson 端的驱动与 DDS 配置保存在 Jetson 部署目录中，本目录没有复制。REGRIND 旧相机启动链的退役配置已按用户要求删除。迁移、删除记录和逐文件 hash 见 [provenance.json](../provenance.json) 的 `post_migration_changes`。
