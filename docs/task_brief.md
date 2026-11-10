# robot-perception 独立仓库：AI 协作任务契约

本文按 `ai-project-workflow` 的任务契约与执行授权模板填写，用于建立跨任务共用的 `robot-perception` 仓库；用户已同意把通用标定与物体感知独立维护。

用户本轮已明确批准阶段 A 的本地迁移及离线消费者接入。package、共享 imports/resources、测试与通用文档已迁移，消费者文档已切换到本仓库并完成离线入口验证；旧源实现曾保留作显式回退，已于 2026-11-10 从 REGRIND 删除。本轮不包含阶段 B、新模型、真实硬件 I/O、删除旧实现、原始数据搬迁、远端创建或发布；B 与实时接入仍需另行批准。

## 目标与边界

仓库位于 `robot-perception`，负责 camera adapter → 通用标定、坐标与时间 → detection/segmentation/6D pose/tracking → 带版本的有效观测。机器人控制、策略、完整 teleop、物体重建与 sysID 不在范围内。

- `wuji-hand-teleop` 继续负责设备与遥操作生命周期，提供相机和状态数据。
- REGRIND（`regrind-vision-deploy`）等任务仓库负责任务与策略并消费观测；迁移后的核心逻辑长期只维护一份。
- tag-to-link、hinge 等任务侧几何与物体 mesh 由任务或资产侧定义，通用模块只消费声明了 frame 与版本的输入，不以它们为平台前提。

保护指标：现有标定行为与测试容差、源数据完整性、坐标与时间语义、candidate 与 approved 结果的区分不退化；离线标定不依赖 ROS、CUDA 或模型权重。

## 已知事实与待验证项

- 迁移来源为 `regrind-vision-deploy` 的 `docs/scissors-camera-perception` 工作树（HEAD `b03026b860d941e78fb7c786111f9674e6cc4f4c`），包含未提交实现。已按实际磁盘冻结来源 SHA，见 [provenance.json](../provenance.json)；新维护入口为[package](../src/robot_perception/calibration/)和[通用流程](calibration_workflow.md)；REGRIND 中的旧实现从未提交，已于 2026-11-10 删除，本仓库为唯一副本。
- 2026-11-09 运行的 36 项离线测试全部通过（13 项 CLI、23 项外参/UI），3 幅真实棋盘图均检测到 88 个内角点。
- 标定板为 DFOPTIX Q12-290-20 普通棋盘，12×9 方格、11×8 内角点，单格 20 mm 由用户实测确认，量具与不确定度未记录。D435 已短时出图，但物理外参、时间精度与 UI 现场使用均未验证。
- 能力缺口：旧 CLI 的内参拟合只支持 ChArUco，capture 要求 aligned depth；新 UI 只做固定相机的 RGB-only 棋盘外参，机器人位姿由文件导入；移动相机和自动读取机器人状态未实现，迁移不补齐。
- 外部 REGRIND 任务背景文档 `REGRIND/docs/vision-deploy/object_recognition_algorithm_options.md` 中的 SAM 3、FoundationPose 等未经验证，不是已定选型；目前没有物体感知基线。该任务侧评估不迁入本包。
- 本轮已验证：在仓库外 cwd、清空 `PYTHONPATH` 后离线构建并安装 wheel，全部 36 项继承测试与新增迁移测试通过；真实 fixture 仅经显式环境变量读取，package import 不加载 Qt/ROS。跨任务观测共用仍待消费者接入检验，不据此宣称已有物体观测实现。
- 未验证解释：软件测试通过或模型官方指标都不能证明实机精度与延迟。

## 阶段与验收

A 先行；本轮已授权的 C 范围仅为标定消费者的离线接入与文档切换。B 和 C 的物体观测或实时部分不在本轮授权内；物体观测接入须先有 B 的离线基线。parent 核对磁盘证据后决定是否向用户申请后续阶段。

### A：标定包迁移（首轮范围）

本轮从仅有 `.git`、main unborn 的目标目录开始；parent 已冻结源文件清单（含 untracked）、hash、dirty 状态和 36 项离线测试结果。按实际磁盘迁移，整理为 `src/robot_perception/` package、稳定的 CLI/UI 入口、`tests/` 与 `pyproject.toml`。保留现有行为，不重写数值算法，不增加控制功能。

- 输入：`scripts/vision_calibration/` 全部文件、`calibration_workflow.md` 和旧文档中的消费者链接；`data/vision_calibration/` 只作外部只读 fixture。`packages/tianji_control/`、策略/训练代码与 `repos/` 不修改。
- 验收：36 项行为测试全部保留，覆盖与容差不降低，失败数为 0；仅本地缺少真实 fixture 时可显式 skip 对应项，synthetic 测试不可 skip。在仓库外 cwd、无 ROS/硬件/GPU/网络时，能安装并导入核心、启动 CLI help 与 offscreen UI、打开旧 session、求解 synthetic X/Y、保存新 revision，并拒绝损坏、覆盖和 stale export。
- 基线与规则：以现有 synthetic 结果与测试容差为基线，源 session、PNG 与 hash 不变。K/D、m/rad、变换方向、角点顺序、平面双解确认、train/heldout 划分和失败记录不得被静默改写；格式升级须带显式版本并兼容读取旧数据。
- 依赖与数据：Qt、ROS 与 GPU 依赖分层，数值核心不隐式加载它们。按 2026-11-10 用户决定，相机、标定板与 DDS 配置在 `config/` 中版本化；raw data、session 与标定记录放在本仓库 `data/`，只在本机、不进 git；模型权重、凭据和 build/venv 不进 git。数据以显式路径、版本和 hash 引用，测试不硬编码本机路径；复用代码须核对来源与 license。

### B：物体感知离线基线

对象与资产版本、数据集与 train/heldout split、模型及 license、GPU 与下载预算、数值阈值均待用户冻结。先选一个可核验的基线，不盲目叠加模型；model-based pose 先用人工核对的 mask 和米制 mesh 单独检查 pose，再接入自动 segmentation。在本机按固定标注与分母报告检测/分割成功率、位姿平移与旋转误差、跟踪丢失率、延迟分布和资源用量，失败、对称物体与无有效深度的样本分开报告。依赖 AprilTag 等标记的结果须标明来源；标记是经标定的参照，不是独立真值。

### C：消费者接入与维护权切换

本轮已完成 REGRIND 标定消费者的文档切换与安装入口离线验证；通用流程和契约只在本仓库维护，任务侧引用验收后的标定版本，不复制通用契约。跨仓库文档链接仅用于本机相邻 checkout，不是已发布远端。旧源实现已删除，新包不调用旧源码；物体观测与实时接入未实施，后者须另行批准 camera-only 工单。删除旧实现、搬迁数据或改动服务须有精确清单并单独授权。

### 共用观测契约

每条观测包含 source timestamp 与 clock domain、frame_id、内参/标定/算法/资产版本、object ID 和有效性或失败原因；有 pose 时写明变换方向与单位。confidence 与 covariance 只在有明确定义时输出；过期、丢失或未标定的观测标为无效，不用 identity pose 填充，也不冒充最新有效状态。

## 权限与分工

A 已获批，writer 可在已核对的新仓库内自主完成复制、package 化、离线测试与文档整理；本轮允许使用已装依赖进行离线 wheel build 与临时 venv 安装，不允许全局安装或联网补依赖。GPU 与模型下载、长时任务、真实相机或机器人 I/O、安装缺失依赖、创建远端和发布各需单独批准。Git stage 不自动进行，每次 commit（含 amend）与 push 分别取得明确许可。任何阶段都不做隐式使能或运动，不自动覆盖驱动配置或正式 TF，不把 candidate 标为 approved，不复制凭据，不破坏已有 dirty/untracked 内容。

- 用户 / owner：决定目标、验收阈值、硬件操作、资源与发布；亲自验收画面、参照和实际标定结果，判定阶段是否通过。
- Parent：编排有边界的任务并负责验收，核对磁盘上的产物、测试证据与剩余风险。
- Astra：package writer 完成 A 后，由唯一 integration writer 定向更新已列明的消费者文档并验证入口，不重做 package；模型按全局 `ai-project-workflow` 绑定，本仓库不另设 agent 配置。
- Reviewer：仅在用户要求并指定模型时启用，以 fresh context 只读核查实际 diff、需求与证据；作者自检不能代替独立审核。

同一 worktree 同时只有一个 writer，parent 应用修复也算写入；未经批准不并行修改多个仓库。writer 完成后按下文交回验收证据。

## 资源、交付与停止条件

- 资源：现有 workstation 与已安装的 Python、NumPy、OpenCV、SciPy、PyQt5，用于 A 的 CPU 离线验证；A 不使用 GPU 或 Jetson。
- 共享限制：现有数据只读；未经批准不停止既有进程，不写机器人、ROS/DDS、网卡配置或其他仓库。
- 交付物：A 的产物是 `~/robot-perception` 中的 package、测试、模板、简洁 README 与通用流程；本契约的唯一维护位置为本文，原流程、契约与脚本说明已改为 canonical 指引；旧源码已于 2026-11-10 删除。
- 验收证据：实际改动路径与源/目标清单 hash、运行命令及 pass/fail/skip、安装/入口与旧 session 兼容结果、只读真实图像检查和未验证项；分别标明“迁移完成”“软件通过”“硬件未验收”。
- 停止条件：目标目录有未知内容、源文件并发变化、license 不明、测试回归、需要改变 schema、越出授权或扩大目标时，保留现场并请求决定。

当前阶段：本地标定迁移、消费者文档切换与离线入口验证已完成；parent 已独立验证离线 wheel 安装、42 项测试、旧空 session 和两种模式的旧含样本 session 兼容性，并核对源数据 hash，用户尚未签收阶段。2026-11-10 起，REGRIND 中的旧实现已删除；按用户决定，相机相关配置迁入 `config/`，标定原始记录原样迁入本仓库 `data/` 并从 REGRIND 移除，退役的旧相机配置和被取代的板记录随后删除；标定板型号确认为 DFOPTIX Q12-290-20。新仓库 main 未 commit、没有 remote，也没有开源发布许可。本轮无独立审核，软件通过不等于物理标定通过，物体感知尚未实现。待后续批准：B 的基线、真实 I/O、远端 owner、可见性、license 与发布方式。
