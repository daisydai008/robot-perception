# robot-perception

通用标定的 Python package：ChArUco 内参候选、AprilTag 角点检测，以及固定相机的 checkerboard 外参工作台。当前只支持已知板参照与 eye-to-hand 两种外参模式，不控制机器人、不发布 TF、不启动相机驱动。所有求解结果保留 `candidate_not_approved` 或 `failed_diagnostic_not_approved` 状态；物体 detection/segmentation/pose/tracking 尚未实现。

## 离线安装

要求 Python 3.10+、NumPy 1.x、OpenCV contrib（含 `aruco`）与 SciPy；UI 另需 PyQt5。系统已装依赖时可用 `--system-site-packages` venv，不修改全局环境。以下命令不下载或升级依赖，不要求安装 `build` 模块；兼容 setuptools 59.6、wheel 0.37、pip 22。`pyproject.toml` 指定 backend，`setup.cfg` 保存旧版 setuptools 可读的 package metadata。

```bash
# 在仓库目录执行；venv、wheel 和验证产物均放到仓库外
export ROBOT_PERCEPTION_REPO="$PWD"
export ROBOT_PERCEPTION_ENV=/tmp/robot-perception-env
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages "$ROBOT_PERCEPTION_ENV"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset PYTHONPATH
"$ROBOT_PERCEPTION_ENV/bin/python" -m pip wheel --no-index --no-cache-dir --no-build-isolation --no-deps "$ROBOT_PERCEPTION_REPO" --wheel-dir /tmp/robot-perception-wheels
"$ROBOT_PERCEPTION_ENV/bin/python" -m pip install --no-index --no-cache-dir --no-deps /tmp/robot-perception-wheels/robot_perception-0.1.0-py3-none-any.whl
source "$ROBOT_PERCEPTION_ENV/bin/activate"
cd /tmp
robot-calibrate --help
robot-extrinsics-ui --help
```

本机 apt OpenCV 可提供 `cv2.aruco`，但没有 pip distribution metadata；pip resolver / `pip check` 不会将它识别成 `opencv-contrib-python`。因此上述系统环境使用 `--no-deps`，需要人工确认实际 import 与功能，不能据此宣称 pip 已安装所有依赖。不要为消除 metadata 提示而全局安装另一套 OpenCV。此机 user-site NumPy 2 与 apt OpenCV ABI 不兼容，需 `PYTHONNOUSERSITE=1` 并选用 `/usr/bin/python3` 创建环境。新的独立环境如缺依赖，应另行批准安装；本轮不联网补齐。

数值层不加载 Qt、ROS 或模型。`[ui]` 是可选 PyQt5 extra，但离线 `--no-deps` 安装仍需系统已有 PyQt5。ROS 仅使用操作人员显式加载的 system ROS，不通过 pip extra 安装；CLI `capture --subscribe` 与 UI 授权按钮仍是实际 I/O 的独立 opt-in，本轮没有真实订阅。没有模型/GPU extra 或模型权重。

## 命令、配置与既有 session

```bash
robot-calibrate --help
python -m robot_perception --help
python -m robot_perception.calibration.calibrate --help
robot-extrinsics-ui
python -m robot_perception.calibration.extrinsics_ui --session "$ROBOT_PERCEPTION_REPO/data/vision_calibration/<session>"
```

`robot-calibrate` 保留 `check-config`、`capture`、`intrinsics`、`detect-tags` 的参数和错误码。唯一维护的 targets 模板随 package 安装；它包含未确认值，默认应被拒绝，不能作为实测配置直接使用。

```bash
mkdir -p /tmp/robot-perception-local
python - <<'PY'
from importlib.resources import files
from pathlib import Path
text = files("robot_perception.calibration").joinpath("targets.example.json").read_text()
with Path("/tmp/robot-perception-local/targets.json").open("x") as stream:
    stream.write(text)
PY
# 预期 exit 2，输出 blocked 与待确认项
robot-calibrate check-config --config /tmp/robot-perception-local/targets.json --formal
```

既有 schema v1 外参 session 可直接通过 `--session` 打开目录，不搬迁或修改原数据；打开会重验输入 hash、PNG、模型与几何，历史结果不恢复，必须重新求解。归档只能保存到新目录，导出不能覆盖现有文件。CLI capture 的 RGB-D raw session 与外参 UI session 是不同格式，不可互换。

Python API 使用 `robot_perception.calibration.extrinsics` 和 `robot_perception.calibration.extrinsics_session.Session`；显式获取 `robot_perception.calibration.extrinsics_ui.Window` 才加载 Qt。准确的 CameraInfo sidecar、diagnostic manifest 与外部参照格式见[外参 UI 使用说明](docs/extrinsics_ui.md)。完整方法与现场验收边界见[通用标定流程](docs/calibration_workflow.md)。

## 配置、数据与测试

相机、标定板与 DDS 配置在 [config/](config/README.md) 中版本化维护。原始图像、session、诊断记录与标定输出保存在本仓库 `data/vision_calibration/`，只在本机、不入 git，以路径与 hash 引用。两者都不随 package 分发，程序不会自动加载；凭据和权重不进仓库。

```bash
# 在已安装 wheel 的环境、仓库外 cwd 执行
export ROBOT_PERCEPTION_FIXTURE_DIR="$ROBOT_PERCEPTION_REPO/data/vision_calibration"
QT_QPA_PLATFORM=offscreen python -m unittest discover -s "$ROBOT_PERCEPTION_REPO/tests" -v
```

测试继承全部 36 项行为检查，新增 package 入口、resource、无 Qt/ROS import 与旧空 session 兼容检查。`ROBOT_PERCEPTION_FIXTURE_DIR` 指向 `vision_calibration` 数据根目录（本机为 `data/vision_calibration/`），包含 `board_diagnostic_20261109_210647/{snapshots.json,color_00.png,color_01.png,color_02.png}` 和 `extrinsics_Q12_setup_20261109_214759/session.json`；该变量同时启用真实图像与历史 session 兼容检查。不设置该变量时仅真实图像项 skip；设置后缺失或损坏会失败，不悄悄 skip。synthetic 核心与 offscreen Qt 测试不得 skip。所有临时结果在 `/tmp`，测试不连接 ROS 或硬件；软件验证不能代替物理标定。

## 方案与历史调研

- [头相机外参方案与厂商 URDF 核查](docs/camera_extrinsic_calibration_options.md)
- [相机场景候选技术栈扫描（2026-10-16）](docs/camera_scenario_scan.md)
- [REGRIND 视觉部署路线图（2026-10-28）](docs/vision_deployment_plan_20261028.md)

以上三份文档及共用 CAPTRA 配图已从 REGRIND 迁入，保留原有历史范围与待核实项；不替代当前通用标定流程或任务侧执行工单。文内指向 REGRIND 的资料链接仅适用于本机相邻 checkout；厂商 URDF 与训练记录仍留在原仓库，相机相关配置已迁入 [config/](config/README.md)，都不是本包运行依赖。

## 来源与授权

迁移来源与源文件 SHA 见 [provenance.json](provenance.json)，本轮范围见[任务契约](docs/task_brief.md)。本地原实现来自 `regrind-vision-deploy` 的实际工作树，包括 untracked 文件。2026-11-10 起，REGRIND 中从未提交的旧脚本已删除，相机标定记录已原样迁入本仓库 `data/` 与 `config/`，逐文件 hash 见 provenance.json；本仓库是唯一维护的实现。已完成离线 wheel 安装、42 项测试及旧 session 兼容验证；真实硬件和物理标定尚未验收。

尚无明确的开源 license 授权，本仓库不赋予 MIT 或其他开源 license，不复制相机 vendor license；第三方仅作为依赖使用，没有 vendor 源码。当前只作已批准的本地迁移与离线接入，公开发布、远端创建、license、真实硬件与阶段 B 均须另行批准。
