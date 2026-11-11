# 物体识别与位姿估计：候选方案评估

本文比较物体识别与位姿估计的候选模块及其接入顺序，适用于本仓库的刚性和 articulated 物体任务，不针对某个具体物体。各模块的输出层次不同：mask 和 box 描述图像中的目标区域，6D pose 才描述物体在空间中的位置与朝向。

## 总体分工与数据流

- **SAM 3**：输出物体 mask，是首选的零样本分割模块。
- **Eagle / LocateAnything**：输出文本指代目标的 box，作为 SAM 3 定位失败时的提示来源。
- **FoundationPose**：由 RGB、深度、内参、mask 和物体米制 mesh 估计相机坐标系下的 6D pose，是位姿主模块。
- **Isaac ROS Pose Estimation**：提供 FoundationPose 的 ROS 2 集成与 NVIDIA 加速实现，属于部署集成选项。
- **CAPTRA**：用于 articulated 物体的分部件位姿与关节角跟踪，刚性物体不需要。

通用数据流为：

```text
RGB → SAM 3 文本提示 → 目标 mask
          ↑ 失败时：Eagle/LocateAnything → box → SAM 3 box prompt
RGB + 原生深度 + 相机内参 + mask + 物体米制 mesh
        → FoundationPose → 目标根位姿（相机坐标系）
        → 已标定相机外参 → 目标根位姿（base_link）
```

## 1. SAM 3：首先测试文本提示分割

官方仓库：[facebookresearch/sam3](https://github.com/facebookresearch/sam3)。它支持文本以及 points、boxes、masks 等提示，图像接口返回 masks、boxes 和 scores。首轮用目标类别名称及同义表述在真实图像上测试，检查是否选中正确实例、关键结构的 mask 是否完整，并记录失败类型与单帧延迟。测试图像应覆盖不同背景、距离、视角和部分遮挡。

SAM 3 的优势是无需先训练检测器，成功时可直接向位姿模块提供 mask。主要风险是目标与背景对比不足、反光区域、遮挡和视频中实例跳变。官方环境要求 Python 3.12+、PyTorch 2.7+ 和 CUDA 12.6+，checkpoint 还需要 Hugging Face 访问授权，应使用独立环境并固定代码与权重版本。

## 2. Eagle / LocateAnything：分割失败时提供 box

官方仓库：[NVlabs/Eagle](https://github.com/NVlabs/Eagle)，当前模型名称为 **LocateAnything-3B**。它接收图像和自然语言查询，输出 grounding box 或 point；box 换算到原图像素后可作为 SAM 3 的视觉提示。

该分支只用于 SAM 3 文本提示没有定位到正确目标的样本。它会增加一个 3B 模型的显存、延迟和故障排查维度，只有实测证明相对 SAM 3 单独分割有稳定收益时才保留。代码与模型采用不同 license，正式部署前需分别核查。

## 3. FoundationPose：由 mask 和 RGB-D 求根位姿

官方仓库：[NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)。model-based 流程输入 mesh、RGB、depth、相机内参和 object mask，首帧调用 `register` 估计位姿，后续帧调用 `track_one` 跟踪；输出是物体相对相机的 6D pose，再经已标定外参进入 `base_link`。

有米制 mesh 时优先采用 model-based 配置，并保证 mesh 尺度、根坐标系和轴向与资产约定一致。model-free 路线需要参考图像和额外神经表示构建，不作为首轮路线。首轮先用人工确认的高质量 mask 验证位姿模块，再切换为自动 mask，从而区分分割误差和位姿误差。刚性物体可用单一根位姿模型；articulated 物体需分部件估计并结合已标定 hinge。

## 4. Isaac ROS Pose Estimation：已有 ROS 2 时的集成选项

官方仓库：[NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)。其中 `isaac_ros_foundationpose` 是 FoundationPose 的 ROS 2 实现，包含 NVIDIA 加速推理、位姿估计和跟踪。它不改变 SAM 3、Eagle 与 FoundationPose 的算法分工，也不会自动解决分割、深度质量或外参问题。

原生离线基线可用且部署确定采用 ROS 2 后，再评估该实现的集成收益。接入前需要核对 Isaac ROS release、ROS 2 distribution、GPU 平台和 TensorRT/Triton 依赖，并明确 mask 如何进入节点图。官方性能数字对应特定硬件和节点配置，不能当作本机实测速度。

## 5. CAPTRA：仅用于 articulated 扩展

CAPTRA（ICCV 2021，arXiv:2104.03437）输入点云，分部件估计 6DoF 位姿与关节角，适合有可动部件的目标。刚性物体用单一根位姿模型即可，不需要 CAPTRA。官方 SAPIEN 预训练权重已恢复到本机，路径见路线文档的 CAPTRA 模型资源一节。

## 推荐验证顺序

1. **SAM 3 文本分割**：验证零样本 mask，记录失败类型与延迟。
2. **Eagle 回退分支**：只在定位失败样本上测试 box → SAM 3，确认实际增益。
3. **FoundationPose 位姿**：先用人工确认 mask 验证 mesh、深度和位姿链路，再接入自动 mask；用留出数据比较平移、旋转和跟踪丢失。
4. **Isaac ROS 集成**：原生基线可用且部署确定采用 ROS 2 后，再比较集成收益、资源占用和端到端延迟。

这一顺序可以逐项定位故障：先判断是否找到目标，再判断能否求出正确位姿，最后判断能否满足部署接口与延迟。
