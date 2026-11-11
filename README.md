# robot-perception

Tianji 机器人相机标定与物体感知的 Python package。头部 D435 的外参已于 2026-11-10 完成并由用户签收：结果见 [extrinsics_base_link_20261110.json](config/cameras/d435/extrinsics_base_link_20261110.json)，方法与验证见[头部 D435 相机标定](docs/calibration/d435_head.md)。物体 detection/segmentation/pose/tracking 尚未实现。包内工具不控制机器人、不发布 TF、不启动相机驱动。

## 离线安装

要求 Python 3.10+、NumPy 1.x、OpenCV contrib 与 SciPy，界面另需 PyQt5。下面的命令用 `--system-site-packages` venv 复用系统已装的依赖，不下载、不升级，也不修改全局环境。

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
```

注意：本机 user-site 中的 NumPy 2 与 apt OpenCV 的 ABI 不兼容，所以必须设置 `PYTHONNOUSERSITE=1`，并用 `/usr/bin/python3` 创建环境。apt OpenCV 没有 pip 元数据，pip 不会把它识别为 `opencv-contrib-python`，因此安装使用 `--no-deps`。数值核心不加载 Qt、ROS 或模型；需要订阅相机的命令使用操作人员显式加载的 system ROS。

## 标定用到的命令

- `robot-extrinsics-ui`：中文 PyQt5 界面，实时订阅已有的 D435 彩色流并检测棋盘，样本表显示每幅的 PnP 重投影 RMS；本次用于检查出厂内参。
- `robot-calibrate rgbd-snapshot`：从已在运行的驱动采集有限帧彩色与原生深度，连同 CameraInfo 和 `/tf_static` 一起保存；必须显式加 `--subscribe` 才会订阅。
- `robot-calibrate arm-check`：离线读取快照、天机关节状态和厂商 URDF，生成机械臂叠图、深度残差图和相机修正量估计。

后两个命令的参数和输出见[机械臂叠图检查](docs/calibration/arm_overlay_check.md)。包内其余命令（`check-config`、`capture`、`intrinsics`、`detect-tags`、`ee-reference`）和界面中的已知板 / eye-to-hand 求解本次标定没有使用，用法见各自的 `--help`。

## 配置、数据与测试

相机、标定板和 DDS 配置在 [config/](config/README.md) 中版本化维护。`data/vision_calibration/` 只在本机、不入 git，目前保存两组测试 fixture：`board_diagnostic_20261109_210647/`（真实棋盘图像）和 `extrinsics_Q12_setup_20261109_214759/`（旧版空 session），以及外参复核记录 `extrinsics_recheck_20261110_2102/`。凭据和模型权重不进仓库。

```bash
# 在已安装 wheel 的环境、仓库外 cwd 执行
export ROBOT_PERCEPTION_FIXTURE_DIR="$ROBOT_PERCEPTION_REPO/data/vision_calibration"
export ROBOT_PERCEPTION_TIANJI_URDF='/home/daisy/regrind-vision-deploy/REGRIND/docs/vision-deploy/Marvin+pro_V3.0_M6_696_SHW5G_TJgripper+urdf.zip'
QT_QPA_PLATFORM=offscreen python -m unittest discover -s "$ROBOT_PERCEPTION_REPO/tests" -v
```

两个环境变量都设置时，89 项测试全部运行。不设置 `ROBOT_PERCEPTION_FIXTURE_DIR` 时真实图像测试 skip，不设置 `ROBOT_PERCEPTION_TIANJI_URDF` 时 2 项厂商 URDF 测试 skip；设置了但文件缺失或损坏时测试失败，不会悄悄 skip。测试不连接 ROS 或硬件。

## 部署与感知路线

[部署与感知路线](docs/vision_deployment_plan.md) 记录当前数据流、已完成事项和下一步；当前首选任务见[水壶识别计划](docs/object_recognition/kettleblack_plan.md)，算法选型见[物体识别与位姿估计：候选方案评估](docs/object_recognition/algorithm_options.md)。

## 来源与授权

迁移来源与源文件 SHA 见 [provenance.json](provenance.json)。本仓库是标定代码唯一维护的实现；REGRIND 中的旧脚本已删除。

本仓库尚无开源 license，不赋予 MIT 或其他开源授权，也不包含 vendor 源码。创建远端、公开发布和 license 选择均须另行批准。
