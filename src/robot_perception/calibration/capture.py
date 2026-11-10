"""Finite camera-only subscriber. ROS imports occur only inside run_capture()."""
from collections import deque
import math
from pathlib import Path
import time

from .common import CalibrationError, positive, require, vision, write_json

STREAMS = {
    "color": ("/cameras/d435/color/image_raw", "/cameras/d435/color/camera_info"),
    "depth": ("/cameras/d435/depth/image_rect_raw", "/cameras/d435/depth/camera_info"),
    "aligned_depth": ("/cameras/d435/aligned_depth_to_color/image_raw",
                      "/cameras/d435/aligned_depth_to_color/camera_info"),
}


def header(msg):
    sec, ns = int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)
    require(sec >= 0 and 0 <= ns < 1_000_000_000 and sec + ns > 0, "Invalid/zero source header timestamp")
    require(bool(msg.header.frame_id), "Missing source frame_id")
    return {"stamp": {"sec": sec, "nanosec": ns}, "stamp_ns": sec * 1_000_000_000 + ns,
            "frame_id": msg.header.frame_id}


def envelope(msg, wall_ns=None, mono_ns=None):
    return {"message": msg, "header": header(msg),
            "receipt_unix_ns": time.time_ns() if wall_ns is None else wall_ns,
            "receipt_monotonic_ns": time.monotonic_ns() if mono_ns is None else mono_ns}


def metadata(item):
    return {key: value for key, value in item.items() if key != "message"}


def info_metadata(item):
    msg = item["message"]
    result = metadata(item)
    result.update(width=int(msg.width), height=int(msg.height), distortion_model=msg.distortion_model,
                  k=list(msg.k), d=list(msg.d), r=list(msg.r), p=list(msg.p),
                  binning_x=int(msg.binning_x), binning_y=int(msg.binning_y),
                  roi={key: getattr(msg.roi, key) for key in ("x_offset", "y_offset", "height", "width", "do_rectify")})
    return result


def decode_image(msg, stream):
    """Honor row stride and endian flag; preserve uint16 depth without scaling."""
    cv2, np = vision()
    require(type(msg.width) is int and type(msg.height) is int and msg.width > 0 and msg.height > 0,
            "Image has invalid dimensions")
    expected = ("rgb8", "bgr8") if stream == "color" else ("16UC1",)
    require(msg.encoding in expected, f"{stream}: expected {expected}, got {msg.encoding}")
    channels, bytes_per_pixel = (3, 3) if stream == "color" else (1, 2)
    require(msg.step >= msg.width * bytes_per_pixel, "Image step shorter than row data")
    require(len(msg.data) == msg.step * msg.height, "Incomplete image buffer")
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    packed = rows[:, :msg.width * bytes_per_pixel].copy()
    if channels == 3:
        image = packed.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    else:
        image = packed.view(np.dtype(">u2" if msg.is_bigendian else "<u2")).reshape(msg.height, msg.width)
        image = image.astype(np.uint16)
    return image


def validate_pair(image, info):
    msg, imeta = image["message"], info_metadata(info)
    require(image["header"] == info["header"], "Image and CameraInfo header must match exactly")
    require((msg.width, msg.height) == (imeta["width"], imeta["height"]), "Image/CameraInfo dimensions differ")
    require(len(imeta["k"]) == 9 and len(imeta["r"]) == 9 and len(imeta["p"]) == 12,
            "Incomplete CameraInfo matrices")
    require(all(math.isfinite(float(x)) for key in ("k", "d", "r", "p") for x in imeta[key]),
            "Non-finite CameraInfo")
    require(imeta["k"][0] > 0 and imeta["k"][4] > 0 and imeta["k"][8] == 1,
            "Uncalibrated CameraInfo K")
    require(bool(imeta["distortion_model"]), "Missing CameraInfo distortion model")
    expected_d = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}
    require(imeta["distortion_model"] in expected_d and len(imeta["d"]) == expected_d[imeta["distortion_model"]],
            "Unsupported or incomplete CameraInfo distortion coefficients; do not reinterpret the model")
    _, np = vision()
    rotation = np.array(imeta["r"]).reshape(3, 3)
    require(np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) and np.linalg.det(rotation) > 0,
            "Incomplete/invalid CameraInfo rectification R")
    require(imeta["p"][0] > 0 and imeta["p"][5] > 0 and imeta["p"][10] == 1,
            "Incomplete/invalid CameraInfo projection P")
    require(imeta["binning_x"] in (0, 1) and imeta["binning_y"] in (0, 1) and
            not any(imeta["roi"].values()), "ROI/binning streams not supported; use full original profiles")
    return imeta


class Pairer:
    """Bounded queues; no reuse of images, exact per-frame CameraInfo matching."""
    def __init__(self, max_delta_ms, queue_size=20):
        require(positive(max_delta_ms), "max pairing delta must be finite and positive")
        self.max_delta_ns = int(max_delta_ms * 1e6)
        self.queues = {key: deque(maxlen=queue_size) for name in STREAMS for key in (name, name + "_info")}
        self.dropped = {key: 0 for key in self.queues}
        self.received = {key: 0 for key in self.queues}
        self.last_used = {key: -1 for key in self.queues}

    def add(self, key, msg):
        item = envelope(msg)
        self.received[key] += 1
        queue = self.queues[key]
        if item["header"]["stamp_ns"] <= self.last_used[key]:
            self.dropped[key] += 1
            return
        if len(queue) == queue.maxlen:
            self.dropped[key] += 1
        queue.append(item)

    def pop(self):
        for color in self.queues["color"]:
            selected = {"color": color}
            stamp = color["header"]["stamp_ns"]
            for key in ("depth", "aligned_depth"):
                queue = self.queues[key]
                if queue:
                    selected[key] = min(queue, key=lambda item: abs(item["header"]["stamp_ns"] - stamp))
            if len(selected) != 3:
                continue
            stamps = [item["header"]["stamp_ns"] for item in selected.values()]
            if max(stamps) - min(stamps) > self.max_delta_ns:
                continue
            for key in STREAMS:
                matching = next((item for item in self.queues[key + "_info"]
                                 if item["header"] == selected[key]["header"]), None)
                if matching is not None:
                    selected[key + "_info"] = matching
            if len(selected) != 6:
                continue
            for key, item in selected.items():
                used = item["header"]["stamp_ns"]
                self.last_used[key] = used
                self.queues[key] = deque((x for x in self.queues[key] if x["header"]["stamp_ns"] > used),
                                         maxlen=self.queues[key].maxlen)
            return selected
        return None


class CaptureWriter:
    def __init__(self, output, config_provenance, max_delta_ms, depth_unit_m, acquisition):
        require(positive(depth_unit_m), "depth-unit-m must be independently confirmed and positive")
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.count = 0
        self.profiles = None
        self.depth_unit_m = depth_unit_m
        write_json(self.output / "session.json", {
            "schema_version": 1, "status": "raw_unverified", "target_config": config_provenance,
            "topics": STREAMS, "max_pair_delta_ms": max_delta_ms, "depth_unit_m": depth_unit_m,
            "acquisition": acquisition,
            "clock_note": "Source header timestamps and local receipt clocks only; exposure/clock synchronization unverified",
            "models": {"color": "raw K,D; distortion_model must be checked before use",
                       "depth": "rectified native depth; do not use raw K,D as rectified projection",
                       "aligned_depth": "registered to color; verify driver mapping and color model"}})

    def save(self, items):
        cv2, np = vision()
        infos = {key: validate_pair(items[key], items[key + "_info"]) for key in STREAMS}
        profiles = {key: {field: value for field, value in info.items()
                          if field not in ("header", "receipt_unix_ns", "receipt_monotonic_ns")}
                    for key, info in infos.items()}
        require(self.profiles is None or self.profiles == profiles, "CameraInfo profile changed during session; start a new session")
        color, aligned = items["color"], items["aligned_depth"]
        require((color["message"].width, color["message"].height, color["header"]["frame_id"]) ==
                (aligned["message"].width, aligned["message"].height, aligned["header"]["frame_id"]),
                "Aligned depth must use the color dimensions and optical frame")
        encoded = {}
        for key in STREAMS:
            image = decode_image(items[key]["message"], key)
            require(np.isfinite(image).all(), "Non-finite image")
            ok, png = cv2.imencode(".png", image)
            require(ok, f"PNG encoding failed: {key}")
            encoded[key] = png.tobytes()
        sample_dir = self.output / f"{self.count:06d}"
        sample_dir.mkdir(exist_ok=False)
        record = {"streams": {}, "pair_delta_ms": {}}
        for key, png in encoded.items():
            name = key + ".png"
            with (sample_dir / name).open("xb") as stream:
                stream.write(png)
            msg = items[key]["message"]
            record["streams"][key] = {**metadata(items[key]), "file": name, "encoding": msg.encoding,
                                      "stored_encoding": "bgr8" if key == "color" else "16UC1",
                                      "width": msg.width, "height": msg.height, "step": msg.step,
                                      "source_is_bigendian": bool(msg.is_bigendian), "camera_info": infos[key]}
            record["pair_delta_ms"][key + "_minus_color"] = (
                items[key]["header"]["stamp_ns"] - color["header"]["stamp_ns"]) / 1e6
        stamps = [items[key]["header"]["stamp_ns"] for key in STREAMS]
        record["pair_delta_ms"]["max_span"] = (max(stamps) - min(stamps)) / 1e6
        write_json(sample_dir / "sample.json", record)  # Only this marks a complete sample.
        self.profiles = profiles
        self.count += 1


def collect(pairer, writer, samples, timeout_s, interval_s, spin, alive=lambda: True, clock=time.monotonic):
    start = clock()
    last_save = -math.inf
    while writer.count < samples and alive():
        remaining = timeout_s - (clock() - start)
        if remaining <= 0:
            break
        spin(min(0.1, remaining))
        now = clock()
        if now - start >= timeout_s:
            break
        items = pairer.pop()
        if items is not None and now - last_save >= interval_s:
            writer.save(items)
            last_save = now
    return writer.count == samples


def run_capture(args, config_provenance):
    # CLI gates and config validation run before reaching any ROS import.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    pairer = Pairer(args.max_pair_delta_ms)
    writer = CaptureWriter(args.output, config_provenance, args.max_pair_delta_ms, args.depth_unit_m, {
        "camera_serial": args.camera_serial, "profile_record": args.profile_record,
        "device_identity_source": "operator_confirmed_not_verified_by_subscriber",
        "target": args.target, "samples_requested": args.samples, "timeout_s": args.timeout_s,
        "interval_s": args.interval_s, "queue_size": 20})
    node = None
    initialized = False
    status = "failed"
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("vision_calibration_capture")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        for key, topics in STREAMS.items():
            node.create_subscription(Image, topics[0], lambda msg, k=key: pairer.add(k, msg), qos)
            node.create_subscription(CameraInfo, topics[1], lambda msg, k=key: pairer.add(k + "_info", msg), qos)
        complete = collect(pairer, writer, args.samples, args.timeout_s, args.interval_s,
                           lambda timeout: rclpy.spin_once(node, timeout_sec=timeout), rclpy.ok)
        status = "complete_raw_unverified" if complete else "timeout_or_shutdown_incomplete"
        require(complete, f"Capture incomplete: {writer.count}/{args.samples}; retained partial session at {args.output}")
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()
            write_json(writer.output / "completion.json", {"status": status, "samples_saved": writer.count,
                       "requested_samples": args.samples, "received": pairer.received, "queue_drops": pairer.dropped})
