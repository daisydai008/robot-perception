# Scissors 头相机外参标定：方案与优先级

> 本文保留 REGRIND 头相机方案与厂商核查记录，已从 `regrind-vision-deploy` 迁入。通用执行流程见[标定流程](calibration_workflow.md)；下文跨仓库资料链接仅适用于本机相邻 checkout，硬件资料不随本仓库分发。

以下按当前外参确定方案的优先顺序排列：**方案 2 → 方案 3 → 方案 1**。三种方案都统一输出头坐标系到 D435 color optical frame 的固定变换；头部运动时，再由头部正运动学计算相机在任务基座坐标系中的位姿。

1. **方案 2：tianji URDF 或文档中的相机外参**：当前首选，优先利用现有模型和资料确定外参。
   - **过程**：获取与当前头部、支架和 D435 安装一致的 URDF、CAD 或标定文档，核对坐标变换链、单位、旋转约定和轴向，确认参数对应 color optical frame 而非外壳或 depth frame；转换到统一坐标链后，用实画叠加和独立几何观测检查。
   - **说明**：该方案成本低且便于复现，可以避免不必要的标定采集和机器人运动。若资料缺失、坐标约定无法确认、实际安装与模型不一致或独立检查不通过，则转入方案 3。
2. **方案 3：ChArUco 经典手眼标定**：当前第二选择，在方案 2 无法给出可用外参时实施。
   - **过程**：先复核相机内参和标定板实测尺寸；将标定板刚性固定在机器人末端并保持头部不动，采集覆盖任务工作区、包含多轴旋转的图像与对应机器人状态；联合求解相机外参和标定板到末端的外参，最后用未参与求解的位姿验证。
   - **说明**：该方法直接测量实际装配关系，能够给出拟合残差和独立验证结果。精度同时受相机内参、标定板安装和机器人运动学影响，因此不能只凭训练图像的 reprojection error 判定通过。
3. **方案 1：视频 + URDF 由同事定位相机**：当前最后选择，仅在前两种方案无法确定外参，或需要快速初始化和目视检查时采用。
   - **过程**：录制包含机器人和任务场景的视频，同时提供匹配的 URDF、关节状态、图像分辨率和相机内参；同事通过渲染—实画对照估计相机位置和朝向，再将结果交给方案 2 或方案 3 检查。
   - **说明**：该过程能较快发现坐标轴、安装方向和视野错误，但人工对齐缺少可靠的精度保证。其结果不得直接进入真机控制链，正式外参仍需几何标定或实测参数验证。

## 厂商 URDF 核查（2026-11-02）

来源文件为 [Marvin+pro_V3.0_M6_696_SHW5G_TJgripper+urdf.zip](../../regrind-vision-deploy/REGRIND/docs/vision-deploy/Marvin+pro_V3.0_M6_696_SHW5G_TJgripper+urdf.zip)（保留在 REGRIND 本地，不入 Git），SHA256 为 `930a00b28141bfc4d25e6eb9414673fb9530dc60bd2e5714382d451ffd4bece9`。包内 URDF 的 robot name 为 `Pro-m6-696`，包含 `Head depth_Link`，其 STL 包围盒为 0.08985756 × 0.02510001 × 0.02499987 m，与 D435 外壳形态一致。

URDF 给出的固定变换如下，长度单位为米，`rpy` 按 URDF roll-pitch-yaw 约定使用弧度：

```xml
<joint name="Head depth_Joint" type="fixed">
  <parent link="base_link" />
  <child link="Head depth_Link" />
  <origin xyz="0 -0.103394958759943 1.24613640461494"
          rpy="1.13446401379639 0 0" />
</joint>
```

令 `M = Head depth_Link`，该文件目前只给出候选机械变换 \({}^{B}T_M\)，并没有给出 D435 color optical frame `C` 相对该 link 的 \({}^{M}T_C\)。URDF 中没有 `color_optical_frame`、其他 optical frame 或相机 sensor plugin，因此不能直接把 `Head depth_Link` 的 origin 当作 RGB 光心。`Head depth_Joint` 被定义为 fixed，文件也未描述头部运动机构；实际头部是否固定及当前支架是否与该型号一致，仍需现场确认。

方案 2 的当前结论是：**已找到候选机械外参，尚未确定正式 color optical 外参。** 候选值即上方 URDF 片段；缺少 \({}^{M}T_C\)，也未做独立验证。REGRIND 旧相机启动链中记录该候选值的 `tianji_d435_extrinsics.yaml` 已按用户要求删除；其中的候选值和缺失项本节均已列出，hash 见 [provenance.json](../provenance.json)。下一步需要 tianji 确认 `Head depth_Link` 的坐标轴和原点定义，并提供或确认 \({}^{M}T_C\)；随后才能形成完整链 \({}^{B}T_C={}^{B}T_M{}^{M}T_C\)，再执行实画叠加和独立几何检查。若厂家无法提供该变换，或实际安装与模型不一致，则按既定优先级转入方案 3。
