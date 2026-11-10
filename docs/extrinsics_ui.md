# 固定相机外参工作台

`robot-extrinsics-ui` 提供中文 PyQt5 desktop UI，用于固定相机、固定头部与 robot base 配置下的 checkerboard 外参候选求解。它不控制机器人、不读取 SDK/FK、不发布 TF、不改相机驱动；启动、导入和 `--help` 不加载 ROS。ChArUco CLI `robot-calibrate` 保持独立，数值算法、输入格式与 CLI 参数不变。package 导入和 `--help` 不加载 Qt；访问 `Window` 或启动 UI 时才加载 Qt。

```bash
robot-extrinsics-ui
# 重新打开原始 session；重新验证输入，不恢复历史求解结果
robot-extrinsics-ui --session /path/to/session
```

依赖现有 Python 环境中的 PyQt5、OpenCV、NumPy、SciPy。离线使用不需要 ROS；仅显式勾选授权并按“连接 ROS”时才加载 `rclpy` / `sensor_msgs`，以 BEST_EFFORT 订阅已有 `/cameras/d435/color/image_raw` 和 `/cameras/d435/color/camera_info`。不启动 driver 或 SSH，不需要 native/aligned depth，alignment 可保持关闭。

## 使用顺序

1. **设置。** 明确 B/base、C/color optical、Q/board、E/end effector frame。DFOPTIX Q12-290-20 预填为 12×9 方格、11×8 内角点、单格 0.020 m；黑白格区域 240×180 mm、整板 290×230 mm、厚 3.2 mm 只是规格，板记录见 [config/boards/](../config/boards/dfoptix_q12-290-20.json)。独立测量尺度并记录后才勾选确认；未确认尺度或 raw model 时仅检测角点，不计算米制 PnP。原点是第一个内部角点，z=0 为印刷面，不是板外角，也不自动加厚度偏置。填写并确认 mm/deg/px 阈值与协议记录，没有默认精度通过值。
2. **导入或预览。** 导入原始 color 图像及下面的 JSON，或显式连接已有 RGB 流。CameraInfo 只接受确认过的 full-resolution raw pinhole `plumb_bob` Brown5 模型、R=I、P=[K|0]、无 ROI/binning；数值、尺寸、source stamp 与 optical frame 必须匹配。拒绝 mixed profile。检测和求解在后台线程执行；实时队列有界，检测只保留最新待处理帧。freeze 时检查 source timestamp 与两个 host receipt 的年龄；source/host 必须有足够一致的 Unix 时钟，检查不等于曝光或机器人同步验收。
3. **冻结与关联。** 每幅选择 train/heldout。图上标记 corner0、+x、+y；必要时反转角点顺序，核对实物上的同一原点/方向。逐幅显式选择并确认 IPPE 平面候选，保留两种解、positive depth 和 reprojection RMS/max；低残差不能消除物理歧义。之后才可关联本幅独立外部位姿 JSON。没有参照的样本只能预览；参照不自动复用。剔除必须填写原因。尺度、模型、frame、模式或原点几何改变需新建 session，不能混合历史捕获。
4. **求解与验证。** 已知板模式需每幅独立 `T_B_Q`，至少 3 train + 2 heldout；由 train 的 `T_B_Q inv(T_C_Q)` 取 SO(3)/平移均值。Eye-to-hand 模式要求板刚性装到末端，输入 `A_i=T_B_E,i`、`C_i=T_C_Q,i`，至少 6 train + 3 heldout、多轴旋转和平移，联合求 `A_i Y = X C_i`，其中 `X=T_B_C`、`Y=T_E_Q`。拒绝重复姿态、单轴或缺乏可观测性的手眼数据；数量只是数值下限。首次求解前冻结 split 与协议记录，此后不允许改 split/追加样本；重新设计分组需新 session 和协议，重复使用同一 heldout 不能作为新的独立验收。
5. **归档与导出。** 保存到不存在的新目录，每次 revision 不覆盖原始记录。session 含 lossless raw PNG、CameraInfo、source/receipt metadata、输入 hash、角点顺序、IPPE candidates、选择/方向确认、参照和剔除原因。打开时重验 hash、模型、SE(3)、候选与配置，不信任保存的结果。所有输入编辑立即废弃结果，异步求解期间编辑也会丢弃结果。导出独立、不可覆盖 JSON，含方向、X/Y、冻结 split、配置/参照来源和逐幅 closure mm/deg、链预测及 PnP reprojection px、阈值失败明细；不静默删除 outlier。阈值失败仍可导出明确标记的 `failed_diagnostic_not_approved`，成功也仅为 `candidate_not_approved`。

移动相机/头部标定不受支持。桌上单图、软件测试或 GUI preview 均不能推断 robot base 外参，也不能证明真实硬件标定通过。独立尺度、板平整度、参照真实性/不确定度、固定安装、物理朝向和现场验收仍由操作人员负责。

### 可选 system ROS 接入（本轮未验证真实订阅）

离线安装不安装或加载 ROS。仅在获得独立的 camera-only I/O 授权后，操作人员在 system ROS 环境中显式加载本机 ROS setup 与 DDS profile，再启动 UI；已有相机驱动须独立运行。workstation 端 Fast DDS profile 与 2026-11-09 收流时的环境变量见 [config/](../config/README.md)，不随 package 分发，使用前须复核网络地址；凭据不进仓库。本轮迁移没有执行下面的实时步骤。

```bash
# 只在已批准的实时环境执行；网络地址变化时先更新 profile
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOT_PERCEPTION_REPO/config/cameras/d435/fastdds_workstation.xml"
robot-extrinsics-ui
# 仍需在 UI 中显式勾选授权并按“连接 ROS”；启动本身不会订阅
```

## 离线 JSON

CameraInfo sidecar 是下面的对象；`image_header` 与 `camera_info.header` 必须逐字段相同，stamp 必须非零。可额外保存 host receipt 和采集记录，程序会保留它们。示例 K/D 仅解释格式，不可用于真实图像。raw 与 rectified 不能单靠 D=0 区分，仍必须确认驱动语义。

```json
{
  "image_geometry": "raw_color_no_crop_no_resize",
  "image_header": {"sec": 1700000000, "nanosec": 1, "frame_id": "d435_color_optical_frame"},
  "camera_info": {
    "header": {"sec": 1700000000, "nanosec": 1, "frame_id": "d435_color_optical_frame"},
    "width": 640, "height": 480, "distortion_model": "plumb_bob",
    "k": [600,0,320,0,600,240,0,0,1], "d": [0,0,0,0,0],
    "r": [1,0,0,0,1,0,0,0,1], "p": [600,0,320,0,0,600,240,0,0,0,1,0],
    "binning_x": 0, "binning_y": 0,
    "roi": {"x_offset": 0, "y_offset": 0, "width": 0, "height": 0, "do_rectify": false}
  }
}
```

也可选择现有 diagnostic `snapshots.json` 与同目录中对应的原始 `color_00.png` 等文件，不选 overlay。该格式 `snapshots[]` 中需要 `stream="color"`、`file`、`header`、`camera_info`；历史诊断未保存 ROI/binning/raw 声明，必须额外勾选 full raw frame 确认后才能导入，此声明会写入 provenance，不能伪称 manifest 已验证它。

外部参照可粘贴或从 JSON 文件读取，`matrix` 是有限、刚性的 4×4 SE(3)（末行 `[0,0,0,1]`，旋转 det +1）。`from_frame` / `to_frame` 与设置完全一致；平移只接受 metres，不隐式换算。每幅使用独立 `source_record`（例如同一测量记录的不同 sample ID），并确认已停稳且与冻结图像同一姿态；缺失参照没有默认 identity。

```json
{
  "units": "m",
  "from_frame": "board",
  "to_frame": "base",
  "matrix": [[1,0,0,0.4],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]],
  "source_record": "独立测量记录#pose-001",
  "settled_same_pose": true
}
```

此矩阵仅示意格式。Eye-to-hand 的 `from_frame` 应为设置中的 E frame；不能将 TCP、wrist 或法兰名字互换。

### Eye-to-hand：由天机关节记录生成参照

`robot-calibrate ee-reference` 是纯离线工具，不导入 `tianji_control`、不连接机器人。天机右臂使用 B=`base_link`、E=`Link7_R`（最后一个运动 link，不是 TCP）；UI 设置中的 frame 必须同名。板刚性安装到末端，安装变换 `T_E_Q` 仍作为未知量求解；厂商 TJ 夹爪与实机 Wuji 手的差异不参与这条 FK 链。

1. 每次停稳并冻结图像后，在保持同一姿态时读取 `tianji_control` 的 `state`，将 UI 样本 ID 与完整状态输出存入文本，每行一个样本。例如：
   ```text
   000000 arm=right frame=123 mode_raw=0 commanded_state_raw=1 error_raw=0 position_degrees=[0, 0, 0, 0, 0, 0, 0]
   ```
   此处零角度仅为格式示例，不是建议运动目标。也接受带 `sample=N/M`、`counter=...` 的 CLI 输出；空行和以 `#` 开头的注释忽略。关节 1–7 顺序、单位 degrees；ID 不重复且仅用字母、数字、`_-.`（以字母或数字开头）。每条 `arm` 须匹配，若有 `error_raw` 必须为 0。
2. 在确认**每条状态都对应已停稳的同一幅冻结图像**后运行：
   ```bash
   robot-calibrate ee-reference \
     --urdf '/path/to/Marvin+pro_V3.0_M6_696_SHW5G_TJgripper+urdf.zip' \
     --base base_link --ee Link7_R --arm right \
     --samples /path/to/states.txt --out-dir /path/to/ee-references \
     --confirm-settled-same-pose
   ```
   URDF 可为 `.urdf` 文件或原位读取的 zip（不解压、不复制厂商材料）；zip 有多个 URDF 时必须用 `--urdf-member` 指定。链须含恰好 7 个角度关节，按链顺序映射并记录 joint names，检查 URDF 限位。全部输入验证后才写 `<sample_id>.json`，已有文件绝不覆盖；I/O 中断可能留下部分文件，须人工核对。未传确认参数则拒绝写入，参数不能证明现场同步。
3. UI 选中对应样本，核对并确认角点方向与 IPPE 候选后，点击“读入参照 JSON 文件”，再点“应用当前样本”；逐幅关联，不自动批量匹配。

输出为米制 `T_B_E`，包含原始状态行、URDF 路径/member、XML 字节 SHA256 与 `fk_from_vendor_urdf_not_validated_against_robot` 来源状态。**FK 正确性依赖厂商 URDF 与实机一致，URDF 关节零位/正负号与 SDK 的约定尚未对机器人验证。** 限位检查与软件交叉验证不能证明这一点；生成的 JSON 仍只是候选参照输入，不代表物理标定通过。

## 离线验证

```bash
# 在已安装 package 的环境运行；显式路径仅用于可选真实图与旧 session 检查
export ROBOT_PERCEPTION_FIXTURE_DIR=/path/to/robot-perception/data/vision_calibration
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen python -m unittest discover -s /path/to/robot-perception/tests -v
```

维护的 unittest 覆盖 synthetic X/Y 方向、train/heldout 隔离、退化、SE(3)/模型/尺度/单位、IPPE/角点反转、session 损坏与禁止覆盖、split 冻结、fake live exact matching/新鲜度、offscreen Qt 导入/冻结/参照/保存/打开/异步求解/导出及过期结果失效。测试不连接 ROS 或真实硬件；真实 subscription、断流、时钟和现场精度需要另行授权验证。
