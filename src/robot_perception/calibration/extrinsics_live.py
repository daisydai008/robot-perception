"""Optional RGB-only ROS subscription. Importing this module never imports ROS."""
from collections import deque
import threading
import time

import cv2
import numpy as np

from .common import require
from .extrinsics import camera_info, profile, validate_header


class RGBMatcher:
    """Exact stamp/frame/dimension pairing with bounded queues, latest match only."""
    def __init__(self, capacity=20):
        self.images = deque(maxlen=capacity)
        self.infos = deque(maxlen=capacity)
        self.last_stamp = -1
        self.locked_profile = None

    def add_image(self, image, header, receipt_unix_ns=None, receipt_monotonic_ns=None):
        validate_header(header)
        self.images.append({"image": image, "image_header": header,
                            "image_receipt_unix_ns": time.time_ns() if receipt_unix_ns is None else receipt_unix_ns,
                            "image_receipt_monotonic_ns": time.monotonic_ns() if receipt_monotonic_ns is None else receipt_monotonic_ns})

    def add_info(self, info, receipt_unix_ns=None, receipt_monotonic_ns=None):
        camera_info(info)
        p = profile(info)
        require(self.locked_profile is None or self.locked_profile == p, "Live CameraInfo/profile changed; reconnect/new session required")
        self.locked_profile = p
        self.infos.append({"camera_info": info,
                           "info_receipt_unix_ns": time.time_ns() if receipt_unix_ns is None else receipt_unix_ns,
                           "info_receipt_monotonic_ns": time.monotonic_ns() if receipt_monotonic_ns is None else receipt_monotonic_ns})

    def pop(self):
        for image in sorted(self.images, key=lambda x: validate_header(x["image_header"]), reverse=True):
            stamp = validate_header(image["image_header"])
            if stamp <= self.last_stamp:
                continue
            info = next((i for i in self.infos if i["camera_info"]["header"] == image["image_header"]), None)
            if info is None:
                continue
            ci = info["camera_info"]
            require(image["image"].shape == (ci["height"], ci["width"], 3), "Live image/CameraInfo dimensions differ")
            self.last_stamp = stamp
            self.images = deque((i for i in self.images if validate_header(i["image_header"]) > stamp), maxlen=self.images.maxlen)
            self.infos = deque((i for i in self.infos if validate_header(i["camera_info"]["header"]) > stamp), maxlen=self.infos.maxlen)
            return {**image, **info, "source": "live", "image_geometry": "raw_color_no_crop_no_resize",
                    "provenance": {"image_topic": "/cameras/d435/color/image_raw",
                                   "info_topic": "/cameras/d435/color/camera_info",
                                   "clock_note": "Source + host receipt only; not exposure/robot synchronisation"}}
        return None


class LiveSource(threading.Thread):
    def __init__(self):
        super().__init__(name="extrinsics-rgb-subscription", daemon=False)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None

    def take(self):
        with self.lock:
            packet, error = self.latest, self.error
            self.latest = None
            self.error = None
        return packet, error

    def stop(self):
        self.stop_event.set()
        self.join()  # spin_once is bounded; destroy only this node/context.

    def run(self):
        node = context = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image

            context = Context()
            rclpy.init(args=[], context=context)
            node = rclpy.create_node("fixed_camera_extrinsics_preview", context=context)
            matcher = RGBMatcher()

            def header(msg):
                return {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec),
                        "frame_id": msg.header.frame_id}

            def on_image(msg):
                wall, mono = time.time_ns(), time.monotonic_ns()
                require(msg.encoding in ("rgb8", "bgr8"), "Only rgb8/bgr8 supported")
                require(msg.width > 0 and msg.height > 0 and msg.step >= msg.width * 3 and
                        len(msg.data) == msg.step * msg.height, "Invalid RGB stride/buffer")
                rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
                image = rows[:, :msg.width * 3].copy().reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                matcher.add_image(image, header(msg), wall, mono)

            def on_info(msg):
                wall, mono = time.time_ns(), time.monotonic_ns()
                info = {"header": header(msg), "width": int(msg.width), "height": int(msg.height),
                        "distortion_model": msg.distortion_model,
                        "k": list(msg.k), "d": list(msg.d), "r": list(msg.r), "p": list(msg.p),
                        "binning_x": int(msg.binning_x), "binning_y": int(msg.binning_y),
                        "roi": {k: getattr(msg.roi, k) for k in ("x_offset", "y_offset", "height", "width", "do_rectify")}}
                matcher.add_info(info, wall, mono)

            qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(Image, "/cameras/d435/color/image_raw", on_image, qos)
            node.create_subscription(CameraInfo, "/cameras/d435/color/camera_info", on_info, qos)
            while not self.stop_event.is_set() and context.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                packet = matcher.pop()
                if packet is not None:
                    with self.lock:
                        self.latest = packet
        except Exception as exc:
            with self.lock:
                self.error = f"ROS RGB 连接失败/停止: {exc}"
        finally:
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()
