# 相机场景候选技术栈扫描（2026-10-16）

> 本文是迁入的历史调研，保留原有核实状态与限制，不代表当前硬件选型或性能已验证；后续 D435 方案背景见[部署与感知路线](../vision_deployment_plan.md)。

## Summary

本报告为"tianji 臂 + Wuji Hand 2 用剪刀"任务在相机场景配置维度上做横向扫描，作为路线图附录的候选技术栈清单。覆盖五个维度：相机硬件形态、布置拓扑、6DoF 位姿方法谱系（2023–2026）、articulated 角度估计、仿真侧相机建模。**重要限制说明：本次运行中 web_search 工具不可用，全部内容基于模型训练期知识（约至 2024 年中–2025 年初），未做实时检索核实**；所有 license、实时性数字、代码链接均标注核实状态，引用前应逐条验证。核心结论：FoundationPose 主线与 RGB-D 硬件强耦合，而剪刀薄金属刃面是各类深度相机的最差工况，这是最需要在附录中显式记录的风险；marker-based 冗余观测与腕载相机是与主线互补性最强的两条支线。

## 前置声明（核实状态约定）

- 【已核实】= 模型知识中有较高置信度的公开事实（论文存在、机构、大致年份），但未在本次运行中联网复核。
- 【待核实】= 具体数字、URL、license 细节，引用前必须人工验证。
- 全局：本报告无任何一条经过本次实时网络验证，因运行环境无搜索工具。

---

## Findings

### 维度 1：相机硬件形态（对"掌内遮挡下剪刀开合角观测"的适配性）

1. **单目 RGB（普通工业相机 / 消费级 webcam）** — 成本最低、帧率/分辨率自由度最大，可搭配 learning-based 单目位姿估计（如 MegaPose、GigaPose 类 render-and-compare 均只需单目 RGB）；但开合角观测在单视角下退化为投影角，近正面视角下角度可观性极差。关键约束：需要 CAD 模型或 category 先验；无尺度/深度真值。【已核实：方法存在性】
2. **被动立体（ZED 2/2i/ZED X，Stereolabs）** — 已暂定的方案。提供 depth + SDK 内的 positional tracking 工具链；但被动立体对**无纹理、反光薄金属刃面**输出深度不可靠或缺失，剪刀刃恰是此形态。关键约束：依赖 NVIDIA GPU + 专有 SDK（闭源）；ZED X 需 GMSL2 载板。【已核实：产品形态与反光物弱点为公知】【待核实：SDK license 条款】
3. **RGB-D 主动立体/结构光（RealSense D435/D455、Orbbec Femto 系列）** — 主动红外散斑缓解无纹理问题，但**无法缓解镜面反光**，金属刃面 depth dropout 依旧；D435 全局快门更适合运动场景，D455 基线更远适合 >1m。Femto Bolt/Mega 是 Azure Kinect DK 停产（2023 年官宣停产）后的主要替代。关键约束：多机同时工作存在红外干扰（需硬件同步/分时）；RealSense SDK 为 Apache-2.0【已核实】，Femto 为 Orbbec SDK【待核实】。【已核实：Azure Kinect 停产】
4. **ToF（Orbbec Femto Mega、Lucid Helios2 等，Azure Kinect 替代品）** — 深度由飞行时间计算，对低纹理略好于被动立体，但镜面反射导致多径/丢失问题同样存在；优势是多机干扰可用分时快门管理、深度延迟低。关键约束：分辨率普遍低于立体方案；近距离精度档位需确认。【待核实：各型号具体规格】
5. **多目环视 RGB（3–4 台普通相机 + 软/硬同步）** — 不依赖主动深度，用多视角几何或多视角 learning 融合；对遮挡鲁棒性最好（总有一台能看到刃口），且规避了金属深度失效问题。关键约束：外参标定与同步工程量大；需要多机位安装空间。【已核实：方案形态】
6. **腕载相机（eye-in-hand）** — 近距离、高分辨率观察剪刀手柄-刃根部，对掌内遮挡场景提供固定相机看不到的视角；但 Wuji Hand 手指本身会周期性遮挡视野，且随臂运动引入 motion blur。关键约束：小型化模组选型（如 OAK-FFC、微型 USB 模组）、线缆管理、外参随臂运动（由臂正运动学补偿）。【已核实：方案形态】
7. **高速相机 / 事件相机（如 Prophesee Metavision）** — 剪刀开合是低速自由度（人手操作量级 <10 Hz），常规 60–120 fps 已足够；事件相机的微秒级时间分辨率对本任务属于过剩能力，仅在未来高速操作时才相关。关键约束：事件相机生态与位姿估计管线不成熟、成本高。【已核实：定性判断】

### 维度 2：相机布置拓扑

1. **固定单视角** — 最简，但对"掌内剪刀"天然不成立：手掌+手指对刃部的遮挡率极高，单视角下开合角观测会长时间丢失，只能依赖跟踪器短期外推。约束：只建议作为 debug/基线配置。
2. **固定多视角（2–4 台，对侧+俯视）** — 主流操作台拓扑；缓解遮挡的确定性手段是视角冗余，建议至少两台成 60–90° 夹角正对操作区。约束：同步（软同步 PTP/硬同步 genlock）、外参标定（ChArUco 板）、算力随路数线性增长。【已核实：常规做法】
3. **腕载 + 固定混合** — 固定相机给全局 root 6DoF，腕载相机补近距离开合角观测；两者信息在滤波层融合。约束：双时间基准对齐、腕载视野内手指自遮挡。【已核实：方案形态】
4. **遮挡缓解的思路清单** — (a) 视角冗余（上述）；(b) 跟踪器外推 + 协方差上报（FoundationPose/BundleTrack 均输出可降权的位姿假设）；(c) learned amodal/遮挡补全（如 HOLD、G-HOP 类 hand-object 重建先验，见维度 3）；(d) 触觉/本体接触事件触发位姿修正（见维度 3 多模态条目）。【部分已核实：各思路均有文献支撑，具体见下文引用】

### 维度 3：6DoF 位姿估计方法谱系（2023–2026）

1. **FoundationPose（NVIDIA，CVPR 2024 Highlight，arXiv:2312.08344）** — 主线方案；RGB-D 输入，支持 model-based 与 NeRF/model-free 两模式，tracking 模式在公开 benchmark（YCBInEOAT）上 SOTA。**关键约束：依赖可用 depth——反光刃面 depth 缺失会直接退化精度；license 为 NVIDIA Source Code License（非商业受限）**【待核实：license 具体条款与 tracking 帧率，论文宣称近实时，社区报告 tracking 模式约 10–30 Hz（RTX 3090 量级）】。代码：https://github.com/NVlabs/FoundationPose【已核实：仓库存在】。
2. **BundleTrack（CMU，IROS 2022，arXiv:2203.13234）** — 无需 CAD 的 RGB-D 实例级 keypoint-memory 跟踪；**速度是主要短板（论文报告约 1.5 s/帧量级，非实时）**，适合离线数据标注而非在线闭环。代码开源（MIT/Apache 待核实）：https://github.com/wenbowen123/BundleTrack【已核实：仓库存在】。
3. **BundleSDF（NVIDIA，CVPR 2023，arXiv:2303.14158）** — RGB-D 在线重建（neural object field）+ 位姿图优化联合估计；重建与跟踪一体，适合"先扫后跟踪"流程；近实时（论文报告 ~10 Hz 量级）【待核实：帧率】。license 同 FoundationPose 系列【待核实】。代码：https://github.com/NVlabs/BundleSDF【已核实：仓库存在】。
4. **CAPTRA（ICCV 2021，arXiv:2104.03437）** — category-level articulated 跟踪：点云输入，同时回归各 part 6DoF + 关节角，**PartNet-Mobility/SAPIEN 数据集中包含 scissors 类别**，与本任务直接对口；论文报告约 12 FPS（硬件型号未在论文中明确，闭环适用性待验证）。代码已公开（GitHub，license 待核实）。约束：需 per-category 训练，泛化到真机剪刀需 domain gap 处理。【已核实：论文与 SAPIEN scissors 类别存在】真实剪刀轨迹证据见论文 [Figure 3 配图](assets/captra_real_scissors_fig3.jpeg)。
5. **Render-and-compare 新一代（单目 RGB 即可）** — **MegaPose（HuggingFace，2023–24，arXiv:2312.06870）**：coarse + refiner 两阶段，合成数据训练，BOP 上强基线，代码 happypose 库开源【license 待核实，速率约 Hz 级非高实时】；**GigaPose（CVPR 2024，arXiv:2312.03788）**：主打速度，宣称毫秒级【待核实】；**SAM-6D（AAAI 2024，arXiv:2311.15707）**：SAM 分割 + 模板匹配，精度高但管线重、非实时；**Any6D（2024，arXiv:2409.12176）**：anchor-free 单 RGB 新物体位姿【待核实】。共同点：只需单目 RGB + CAD，天然规避金属深度失效——这是对主线最有价值的正交替代族。
6. **遮挡/hand-object 专项（2023–2024）** — **HOLD（CVPR 2024，arXiv:2311.16448）**：category-agnostic 手-物体重建，从单目视频分解手与物体；**G-HOP（CVPR 2024，arXiv:2404.12396）**：generative hand-object prior，可作为遮挡下的位姿先验。均为重建/先验性质，非实时跟踪器，但可生成训练数据或作遮挡推理模块。【已核实：论文存在】【待核实：代码与 license】
7. **触觉/本体融合多模态状态估计** — 思路：GelSight 类触觉给出局部接触几何，修正被完全遮挡时的物体位姿（Tac2Pose, RSS 2022, arXiv:2202.05545；MidasTouch, CoRL 2022, arXiv:2205.03515）；Wuji Hand 2 若有指尖触觉或可复用关节力/位置信息推断接触状态。约束：非即插即用，需自建融合滤波器；作为主线被遮挡时的降级通道。【已核实：论文存在】

### 维度 4：articulated / 可开合物体角度估计（CAPTRA 之外）

1. **Marker-based（经典，一句话）** — AprilTag 3 / ChArUco / STag：在剪刀两片上各贴一个微型 fiducial，由相对位姿直接解算开合角，精度亚度级、零训练成本，是成本最低的 ground-truth/冗余观测通道；约束：贴标空间、遮挡下丢帧、与"纯视觉"目标冲突（但作为验证手段合理）。【已核实】
2. **ScrewNet（IROS/RA-L 2023，arXiv:2305.09972 附近【待核实编号】）** — category-level 关节参数（screw axis + 角度）从点云序列估计，跨实例泛化；适合"任意剪刀型号"设定。【待核实：代码可用性与实时性】
3. **PARIS（CVPR 2023，arXiv:2302.06340，Stanford）** — part-level 交互式重建：从两状态 RGB-D 观测分解 articulated 物体的 part 几何与关节；非跟踪器，但可用于离线给任意剪刀建 articulated 模型，喂给主线跟踪器。【已核实：论文存在】【待核实：代码】
4. **ArtGS / 可动高斯重建族（2024，如 arXiv:2406.16623 ArtGS【待核实编号】）** — 用 3D Gaussian Splatting 重建可动物体的外观+关节，天然支持 render-and-compare 式角度回归；处于快速迭代期，实时性与工程成熟度未定。【待核实：整体】
5. **学习式 part pose（category-level keypoint / NOCS 扩展）** — ANCSH（2021，arXiv:2101.07474）一脉的 normalized part coordinate 方法，近年有多个后续（如 HS-Pose, CVPR 2023 处理类别级 rigid 6D；articulated 后续工作 2023–2024 若干【待核实具体条目】）；共同点：per-category 训练、对点云质量敏感。
6. **事件相机/高速方案** — 事件相机 6DoF 跟踪研究存在（line/shape-based event tracking 一脉），但无成熟 articulated 物体方案；本任务速度区间不需要，仅列为远期选项。【待核实：具体代表工作】

### 维度 5：仿真侧相机建模

1. **Isaac Sim RTX Camera 传感器** — 提供 RGB/depth/semantic 输出与基于光栅的真实渲染；depth 为理想值，**噪声需后处理注入**（社区常用 Redwood/Kinect 噪声模型叠加）。文档：docs.omniverse.nvidia.com Isaac Sim Sensors–Camera 章节【待核实：具体 URL 与最新版本特性】。【已核实：传感器框架存在】
2. **深度噪声模型** — 经典 Redwood noise model（Nguyen et al., 2012）按距离/入射角建模轴向噪声；结构光缺失建模（反光/边缘 dropout）通常需自定义 mask 启发式；**对本任务关键是"金属刃面 dropout"必须在仿真里显式模拟，否则 sim2real 对 FoundationPose 的 depth 输入会严重高估**。【已核实：Redwood 模型存在；dropout 建模为工程共识】
3. **Domain randomization 相机相关项清单** — 曝光/增益/gamma、白平衡、motion blur（按曝光时间内物体位移卷积）、滚动快门畸变（若用 rolling shutter 相机）、镜头畸变参数、内参抖动、depth 量化与空洞、帧率/延迟抖动（policy 闭环消费对延迟敏感，建议建模固定延迟 + 随机抖动）。【已核实：各项均为 sim2real 文献常规项，如 Tobin et al. 2017 及后续】
4. **延迟与时间戳建模** — Isaac Sim 本身不模拟相机 pipeline 延迟；需在外层 wrapper 注入（曝光+读出+传输+推理 ≈ 30–80 ms 量级，依硬件）。【待核实：具体延迟数字按最终硬件实测】

---

## 对比总结（与 FoundationPose 主线的关系）

- **强互补（建议保留在路线图附录作为冗余/降级通道）**：marker-based 微型 fiducial 通道——提供开合角独立真值与遮挡期观测，成本极低，与任何主线不冲突；腕载 RGB 相机——补固定视角看不到的刃根视角；触觉/本体融合——主线被完全遮挡时的降级估计。
- **正交替代（单目 RGB 族，规避 FoundationPose 对 depth 的硬依赖）**：MegaPose / GigaPose / Any6D 等 render-and-compare 新族 + 多目 RGB 环视——针对"反光薄刃面 depth 不可靠"这一主线最大风险点，是最值得并行验证的备选栈。
- **流程互补（离线建模喂主线）**：PARIS / ArtGS / BundleSDF——给任意真机剪刀建 articulated 或可渲染模型，再交给 FoundationPose tracking 或 part-based 跟踪。
- **直接对口但需 domain gap 工程**：CAPTRA（SAPIEN 含 scissors 类）——分部件跟踪候选中泛化路径最清晰的一条。
- **不建议投入**：事件相机/高速相机（速度区间过剩）、单固定视角（遮挡下不可行）、纯 ToF 升级（对反光刃面无本质改善）。

## 参考文献

以下文献为正文各候选方案的出处，按主题分组；标注【待核实】的条目在引用前须人工复核（本扫描未联网，见 Gaps）。

位姿估计主线与回退：
- FoundationPose（arXiv:2312.08344）：RGB-D 6DoF 位姿估计与跟踪统一框架，官方代码与权重公开，tracking 达实时量级，本路线图根位姿主线。license 为 NVIDIA Source Code License【待核实条款】。
- BundleSDF（arXiv:2303.14158）：RGB-D 在线重建与位姿联合估计，先扫后跟踪，主线回退之一。
- BundleTrack（arXiv:2203.13234）：无需 CAD 的 RGB-D 实例级跟踪，速度慢，适合离线标注而非在线闭环。

articulated 角度估计：
- CAPTRA（arXiv:2104.03437）：category-level articulated 跟踪，官方代码/数据/预训练权重公开，含真实剪刀轨迹，开合角候选 B。
- DICArt：scissors 双部件误差参照（半合成数据），release 与实时性未核实，仅作研究对照。
- PARIS（arXiv:2302.06340）：交互式 part-level articulated 重建，用于离线给真机剪刀建可动模型。
- ANCSH（arXiv:2101.07474）：category-level 关节参数估计先驱方法。
- ScrewNet（2023，编号【待核实】）：点云序列的关节参数估计，跨实例泛化，供多剪刀型号扩展参考。

单目 RGB 替代族（规避 depth 失效）：
- MegaPose（arXiv:2312.06870）：单目 RGB render-and-compare，合成数据训练，BOP 强基线。
- GigaPose（arXiv:2312.03788）：同族，主打速度。
- SAM-6D（arXiv:2311.15707）：SAM 分割加模板匹配的 zero-shot 位姿估计，管线重、非实时。
- Any6D（arXiv:2409.12176）：anchor-free 单 RGB 新物体位姿【待核实】。

遮挡与多模态：
- HOLD（arXiv:2311.16448）：单目视频的手-物体分解重建，可作遮挡推理与数据生成先验。
- G-HOP（arXiv:2404.12396）：generative hand-object prior，遮挡下的位姿先验。
- Tac2Pose（arXiv:2202.05545）：触觉局部接触几何修正物体位姿。
- MidasTouch（arXiv:2205.03515）：触觉全局位姿估计，完全遮挡时的降级通道参考。

蒸馏与仿真基础：
- DextrAH-RGB（arXiv:2412.01791）：状态 teacher 到 RGB student 的蒸馏先例，蒸馏路线的方法来源。
- Nguyen et al. 2012（Redwood depth noise）：深度相机噪声经典模型，仿真深度噪声注入依据。
- Tobin et al. 2017：domain randomization 基础文献，相机相关随机化项清单来源。

## Gaps

1. **许可与数字需逐项核实**：本文可联网核查，但撰写时未逐项验证所有 license（尤其 FoundationPose 的 NVIDIA Source Code License 条款）、实时帧率数字、代码仓库当前状态和部分 arXiv 编号（ScrewNet、ArtGS 标注为待核实），引用前需人工复核。
2. **2025–2026 最新文献缺失**：训练知识截止后新出现的 articulated 跟踪、render-and-compare 后续工作未覆盖；建议后续用关键词 "articulated object pose tracking 2025"、"category-level articulation estimation 2025" 补检索。
3. **Wuji Hand 2 触觉能力未确认**：多模态融合条目的可行性取决于该手是否具备指尖触觉/力反馈，需查硬件规格。
4. **真机剪刀的 depth 失效程度无量化数据**：建议尽早做一次性实验（ZED/RealSense 对实物剪刀刃面的 depth 完整率统计），该数据直接决定主线是否需要单目 RGB 备选栈。
5. 下一步建议：对本报告所有【待核实】项逐条联网复核，并补 2025–2026 文献。
