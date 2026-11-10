"""中文固定相机外参工作台。启动不连接 ROS；不控制机器人、不发布 TF。"""
import argparse
import sys


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    root.add_argument("--session", help="打开已保存的 session 目录（重新验证，不恢复结果）")
    return root


def main(argv=None):
    # Parse first: help/invalid arguments do not require Qt, OpenCV or ROS.
    args = parser().parse_args(argv)
    try:
        from ..ui._extrinsics_window import main as run_window
    except ImportError as exc:
        print(f"ERROR: UI requires PyQt5, OpenCV, NumPy and SciPy: {exc}", file=sys.stderr)
        return 2
    return run_window(args)


def __getattr__(name):
    # Accessing a UI class is an explicit request for the optional Qt layer.
    if name in {"Window", "Detector", "Solver"}:
        from ..ui import _extrinsics_window
        return getattr(_extrinsics_window, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


if __name__ == "__main__":
    sys.exit(main())
