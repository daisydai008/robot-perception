# KettleBlack 水壶识别计划

本文定义黑色水壶的识别与位姿估计计划。水壶是刚性、无铰链的大物体，是当前物体感知的首个验证对象；剪刀等 articulated 任务放在其后。

## 任务关系与现有条件

REGRIND 的 KettleBlack 阶段二需要相机估计水壶状态，再交给策略闭环。本文只覆盖离线识别与位姿基线，不进入控制环。

可直接复用的条件：头部 D435 外参已签收（头部固定、静态使用），内参采用出厂值，彩色流为 848×480@30 rgb8、深度为 640×480 原生深度；快照和叠图工具见[标定工具说明](../calibration/arm_overlay_check.md)。算法候选与深度调研见[候选方案评估](algorithm_options.md)和[相机场景候选技术栈扫描](camera_scenario_scan.md)。

## 资产

模型副本在本机 `data/kettleblack/model/`（不入 Git），逐文件 hash 见同目录 `manifest.json`。来源为 REGRIND 的 `data/kettleblack/simulator/data/hand2_right/kettleblack_meshtip_v1/object`，包含：

- `KettleBlack.obj`：视觉 mesh，供 model-based 位姿估计使用。
- `kettleblack.urdf`：物体坐标系与模型引用。
- `collision_parts/part_000.obj` 至 `part_009.obj`：碰撞分块，供后续仿真或接触检查。

纹理 `KettleBlack.jpg` 已从 RTX（xsuper）上的 NAS 挂载 `/mnt/nist/nas_24/nist/TactiDex/TactiDex_objects/KettleBlack/` 取回，SHA256 为 `8ba9a1e00134f02951ce94a96da3a995138b790b2317507cee47c9da36627f15`；OBJ/MTL 与 NAS 上同目录文件的 hash 一致，MTL 的 `map_Kd` 引用现已满足。同款水壶到货情况、实际尺寸、把手、空壶质量和壶盖固定方式需与实物核对；mesh 的尺度、根坐标系和轴向必须与 URDF 一致。

## 验证顺序

1. **实物与模型核对**：确认水壶到位，测量并记录尺寸、质量和壶盖状态；检查 mesh 尺度、根坐标系和轴向。
2. **SAM 3 文本分割**：以 `kettle`、`black kettle` 等提示在真实 D435 图像上测试，覆盖不同距离、视角、背景和部分手部遮挡，记录实例选择、把手与壶盖区域的 mask 完整性、失败类型和单帧延迟。
3. **Eagle 回退**：只在 SAM 3 定位失败的样本上测试 Eagle/LocateAnything 的 box → SAM 3，确认有稳定收益才保留。
4. **FoundationPose 根位姿**：先用人工确认的 mask 和 `KettleBlack.obj` 验证深度、尺度和位姿链路，再接入自动 mask。输出相机坐标系下的 6D pose，经已签收外参转换到 `base_link`。
5. **集成评估**：原生离线基线可用后，再比较 Isaac ROS Pose Estimation 的集成收益、资源占用和端到端延迟。

水壶无铰链，CAPTRA、开合角估计和双连杆资产不在本计划内。

## 评测口径

数据集和 train/heldout 划分在评测前固定，按固定标注与分母报告检测/分割成功率、位姿平移与旋转误差、跟踪丢失率、延迟分布和资源用量。失败样本、遮挡样本和无有效深度样本分开报告。每次运行归档原始 RGB-D、机器人状态、估计结果、时间戳以及标定、资产和算法版本。

## 交付边界

交付离线识别与位姿结果及未通过项清单。接入 REGRIND 阶段二的实时观测需另行批准 camera-only 工单；观测需带源时间戳、frame_id、标定/算法/资产版本、object ID 和有效性或失败原因。
