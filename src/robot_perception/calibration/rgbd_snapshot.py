"""Finite RGB/native-depth diagnostic snapshot; ROS is imported only by run_snapshot."""
from collections import deque
from pathlib import Path
import time

from .capture import decode_image, envelope, metadata, validate_pair
from .common import CalibrationError, positive, require, vision, write_json

TOPICS = {
    "color": ["/cameras/d435/color/image_raw", "/cameras/d435/color/camera_info"],
    "depth": ["/cameras/d435/depth/image_rect_raw", "/cameras/d435/depth/camera_info"],
    "tf_static": "/tf_static",
}


def static_transform(transforms, target, source):
    """Return T_target_source from a consistent static TF tree (metres, xyzw)."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from .extrinsics import inverse

    parents, edges = {}, {}
    for record in transforms:
        parent, child = record["parent"], record["child"]
        require(isinstance(parent, str) and isinstance(child, str) and parent and child and parent != child,
                "Invalid static TF parent/child")
        t = np.asarray(record["translation"], dtype=float)
        q = np.asarray(record["rotation_xyzw"], dtype=float)
        require(t.shape == (3,) and q.shape == (4,) and np.isfinite(t).all() and np.isfinite(q).all()
                and abs(np.linalg.norm(q) - 1) < 1e-5, "Invalid static TF translation/quaternion")
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = Rotation.from_quat(q).as_matrix(), t
        if child in parents:
            old_parent, old_matrix = parents[child]
            require(parent == old_parent and np.allclose(matrix, old_matrix, atol=1e-9, rtol=0),
                    f"Conflicting static TF for {child}")
        parents[child] = (parent, matrix)
        edges.setdefault(child, {})[parent] = matrix
        edges.setdefault(parent, {})[child] = inverse(matrix)
    for child in parents:
        visited, current = set(), child
        while current in parents:
            require(current not in visited, "Cycle in static TF")
            visited.add(current)
            current = parents[current][0]
    pending, seen = [(source, np.eye(4))], set()
    while pending:
        frame, matrix = pending.pop()
        if frame == target:
            return matrix
        seen.add(frame)
        for neighbor, step in edges.get(frame, {}).items():
            if neighbor not in seen:
                pending.append((neighbor, step @ matrix))
    raise CalibrationError(f"Missing static TF chain: {source} -> {target}")


def validate_args(args):
    require(args.subscribe, "rgbd-snapshot requires explicit --subscribe for camera-only I/O")
    require(type(args.frames) is int and 1 <= args.frames <= 300, "frames must be in [1,300]")
    require(positive(args.timeout_s) and args.timeout_s <= 600, "timeout-s must be finite in (0,600]")
    require(positive(args.depth_unit_m), "depth-unit-m must be independently confirmed, finite and positive")
    require(bool(args.camera_serial.strip()) and bool(args.profile_record.strip()),
            "camera-serial and profile-record must be nonempty provenance strings")
    require(not Path(args.output).exists() and not Path(args.output).is_symlink(),
            f"Refusing existing output: {args.output}")


class SnapshotWriter:
    """Exclusive output, exact per-stream Image/CameraInfo pairing; no RGB/depth sync claim."""
    def __init__(self, output, frames, depth_unit_m, camera_serial, profile_record):
        require(type(frames) is int and 1 <= frames <= 300, "frames must be in [1,300]")
        require(positive(depth_unit_m), "depth-unit-m must be finite and positive")
        require(bool(camera_serial.strip()) and bool(profile_record.strip()), "Missing acquisition provenance")
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.frames = frames
        self.profiles = {}
        self.record = {
            "schema_version": 1, "status": "diagnostic_snapshot_not_calibration", "topics": TOPICS,
            "depth_unit_m": depth_unit_m, "frames_requested_per_stream": frames,
            "provenance": {"camera_serial": camera_serial, "profile_record": profile_record,
                           "identity_source": "operator_confirmed_not_verified_by_subscriber"},
            "clock_note": "Independent streams; source stamps and host receipt clocks do not prove synchronization",
            "streams": {"color": [], "depth": []}, "tf_static": [],
        }

    def save(self, stream, image, info):
        require(stream in self.record["streams"], f"Unsupported snapshot stream: {stream}")
        records = self.record["streams"][stream]
        require(len(records) < self.frames, f"Too many {stream} frames")
        imeta = validate_pair(image, info)
        profile = {k: v for k, v in imeta.items() if k not in
                   ("header", "receipt_unix_ns", "receipt_monotonic_ns")}
        profile["frame_id"] = image["header"]["frame_id"]
        require(stream not in self.profiles or profile == self.profiles[stream],
                f"{stream}: CameraInfo profile/frame changed during snapshot")
        require(not records or image["header"]["stamp_ns"] > records[-1]["header"]["stamp_ns"],
                f"{stream}: duplicate or non-increasing source timestamp")
        cv2, _ = vision()
        ok, png = cv2.imencode(".png", decode_image(image["message"], stream))
        require(ok, f"PNG encoding failed: {stream}")
        filename = f"{stream}_{len(records):06d}.png"
        with (self.output / filename).open("xb") as out:
            out.write(png.tobytes())
        records.append({**metadata(image), "file": filename, "camera_info": imeta,
                        "encoding": image["message"].encoding,
                        "stored_encoding": "bgr8" if stream == "color" else "16UC1"})
        self.profiles[stream] = profile

    def add_tf(self, msg):
        wall, mono = time.time_ns(), time.monotonic_ns()
        for tf in msg.transforms:
            t, q = tf.transform.translation, tf.transform.rotation
            self.record["tf_static"].append({
                "parent": tf.header.frame_id, "child": tf.child_frame_id,
                "translation": [float(t.x), float(t.y), float(t.z)],
                "rotation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
                "stamp": {"sec": int(tf.header.stamp.sec), "nanosec": int(tf.header.stamp.nanosec)},
                "receipt_unix_ns": wall, "receipt_monotonic_ns": mono})

    @property
    def complete(self):
        return all(len(records) == self.frames for records in self.record["streams"].values())

    def check_chain(self):
        frames = {k: v[0]["header"]["frame_id"] for k, v in self.record["streams"].items() if v}
        require(len(frames) == 2, "Both color and native depth frames required")
        return static_transform(self.record["tf_static"], frames["color"], frames["depth"])

    def finish(self):
        require(self.complete, "Snapshot incomplete: not enough frames per stream")
        self.check_chain()
        write_json(self.output / "snapshot.json", self.record)
        return self.record


class SnapshotPairer:
    """Bounded exact-header queues for two independent streams, with no info reuse."""
    def __init__(self, writer):
        self.writer = writer
        self.queues = {key: deque(maxlen=40) for s in ("color", "depth") for key in (s, s + "_info")}
        self.last = {s: -1 for s in ("color", "depth")}

    def add(self, key, msg):
        stream = key.removesuffix("_info")
        if len(self.writer.record["streams"][stream]) >= self.writer.frames:
            return
        item = envelope(msg)
        if item["header"]["stamp_ns"] <= self.last[stream]:
            return
        self.queues[key].append(item)
        for image in self.queues[stream]:
            info = next((x for x in self.queues[stream + "_info"] if x["header"] == image["header"]), None)
            if info is not None:
                self.writer.save(stream, image, info)
                self.last[stream] = image["header"]["stamp_ns"]
                for queue_key in (stream, stream + "_info"):
                    self.queues[queue_key] = deque((x for x in self.queues[queue_key]
                                                  if x["header"]["stamp_ns"] > self.last[stream]), maxlen=40)
                break


def run_snapshot(args):
    validate_args(args)  # Also gate direct callers before any ROS import or filesystem write.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    writer = SnapshotWriter(args.output, args.frames, args.depth_unit_m, args.camera_serial, args.profile_record)
    pairer = SnapshotPairer(writer)
    node, initialized = None, False
    start = time.monotonic()
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("robot_rgbd_diagnostic_snapshot")
        images_qos = QoSProfile(depth=40, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for stream in ("color", "depth"):
            node.create_subscription(Image, TOPICS[stream][0], lambda m, s=stream: pairer.add(s, m), images_qos)
            node.create_subscription(CameraInfo, TOPICS[stream][1],
                                     lambda m, s=stream: pairer.add(s + "_info", m), images_qos)
        node.create_subscription(TFMessage, TOPICS["tf_static"], writer.add_tf, static_qos)
        chain_error = "No frames/TF received"
        while rclpy.ok() and time.monotonic() - start < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, args.timeout_s - (time.monotonic() - start))))
            if writer.complete:
                try:
                    writer.check_chain()
                except CalibrationError as exc:
                    chain_error = str(exc)
                else:
                    return writer.finish()
        raise CalibrationError(f"Snapshot timeout/shutdown: incomplete frames or static TF ({chain_error}); "
                               f"partial PNGs retained at {args.output}; no completed snapshot.json")
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()
