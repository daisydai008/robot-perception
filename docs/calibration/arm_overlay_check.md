# 机械臂叠图与原生深度诊断

本文说明如何用 `rgbd-snapshot` 和 `arm-check` 核对固定 D435 的候选外参：采集彩色与原生深度快照，生成机械臂叠图、深度残差和修正量估计，供人工判断外参是否与实物一致。

## 1. 采集快照

前提：Jetson 上 D435 在运行，workstation 能收到 `/cameras/d435/*`，双臂保持静止。先记录一次关节角，再采集：

```bash
robot-calibrate rgbd-snapshot --subscribe \
  --output /tmp/arm-p1 --frames 15 --timeout-s 30 \
  --depth-unit-m 0.001 --camera-serial 348122072721 \
  --profile-record 'D435 848x480@30 rgb8，原生深度 640x480'
```

`--subscribe` 代表现场相机授权，输出目录请使用新目录。快照包含彩色 PNG、原始 16 位深度 PNG、逐帧 CameraInfo、源与接收时间戳，以及 `/tf_static` 中的相机出厂变换。

## 2. 准备输入并离线核对

`states.txt` 每行写一个姿态 ID 和一条 `tianji_control state` 输出，左右臂各一行：

```text
p1 arm=right frame=123 mode_raw=0 commanded_state_raw=1 error_raw=0 position_degrees=[...7 个角度...]
p1 arm=left  frame=124 mode_raw=0 commanded_state_raw=1 error_raw=0 position_degrees=[...7 个角度...]
```

`camera_pose.json` 取签收外参中的 `d435_color_optical_frame` 一项。随后运行：

```bash
URDF='/home/daisy/regrind-vision-deploy/REGRIND/docs/vision-deploy/Marvin+pro_V3.0_M6_696_SHW5G_TJgripper+urdf.zip'
robot-calibrate arm-check --urdf "$URDF" --base base_link \
  --arm-chain right=Link7_R --arm-chain left=Link7_L \
  --links Link1_R,Link2_R,Link3_R,Link4_R,Link5_R,Link6_R,Link7_R,Link1_L,Link2_L,Link3_L,Link4_L,Link5_L,Link6_L,Link7_L \
  --states states.txt --pose p1=/tmp/arm-p1 \
  --camera-pose camera_pose.json --output /tmp/arm-check-p1 \
  --confirm-static-same-pose
```

多条姿态可重复 `--pose p2=/tmp/arm-p2` 并补充对应状态行。链接列表只含两条臂的 Link1–Link7，手部几何按实际末端单独处理。

## 3. 看结果

- `pose_<id>_overlay.png`：输入外参的叠图，目视检查手臂轮廓是否对齐。
- `pose_<id>_depth_residual.png`：深度点到模型表面的距离，绿 ≤5 mm、黄 5–15 mm、红 >15 mm，灰色为未匹配点。
- `arm_check.json`：按姿态和臂列出距离中位数、p95、修正量，以及留一核对结果。留一核对可判断修正量能否迁移到其他姿态或手臂。

结果用于人工核对。深度噪声、外壳与走线差异都会影响数值，多姿态比单姿态更能说明问题。头部、支架、相机或 URDF 变化后，按同样步骤重新核对。
