"""Offline native RGB-D writer tests; fake ROS-shaped messages only."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import cv2
import numpy as np

from robot_perception.calibration.calibrate import main, parser
from robot_perception.calibration.capture import envelope
from robot_perception.calibration.common import CalibrationError
from robot_perception.calibration.rgbd_snapshot import SnapshotPairer, SnapshotWriter, run_snapshot, static_transform


def messages(image, frame, stamp=1, k=None, rgb=False):
    h, w = image.shape[:2]
    k = np.array([[100, 0, w / 2], [0, 100, h / 2], [0, 0, 1]]) if k is None else k
    hdr = NS(stamp=NS(sec=stamp, nanosec=7), frame_id=frame)
    info = NS(header=hdr, width=w, height=h, k=k.ravel().tolist(), d=[0.] * 5,
              r=np.eye(3).ravel().tolist(), p=np.column_stack((k, np.zeros(3))).ravel().tolist(),
              distortion_model="plumb_bob", binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
    msg = NS(header=hdr, width=w, height=h, encoding=("rgb8" if rgb else "bgr8") if image.ndim == 3 else "16UC1",
             step=image.strides[0], is_bigendian=False, data=image.tobytes())
    return msg, info


def tf_message(parent="color", child="depth", translation=(.02, 0, 0)):
    return NS(transforms=[NS(header=NS(frame_id=parent, stamp=NS(sec=0, nanosec=0)), child_frame_id=child,
                             transform=NS(translation=NS(**dict(zip("xyz", translation))),
                                          rotation=NS(x=0., y=0., z=0., w=1.)))])


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "snapshot"

    def writer(self, frames=1):
        return SnapshotWriter(self.output, frames, .001, "serial", "driver record")

    def fill(self, writer):
        for stream in ("color", "depth"):
            array = np.full((4, 6, 3), [10, 20, 30], np.uint8) if stream == "color" else np.full((4, 6), 1234, np.uint16)
            msg, info = messages(array, stream, rgb=True)
            writer.save(stream, envelope(msg, 10, 20), envelope(info, 11, 21))

    def test_writer_exact_metadata_native_units_and_bgr(self):
        writer = self.writer()
        self.fill(writer)
        writer.add_tf(tf_message())
        result = writer.finish()
        saved = json.loads((self.output / "snapshot.json").read_text())
        self.assertEqual(saved, result)
        self.assertEqual(saved["schema_version"], 1)
        self.assertEqual(saved["status"], "diagnostic_snapshot_not_calibration")
        self.assertEqual(saved["depth_unit_m"], .001)
        color = saved["streams"]["color"][0]
        self.assertEqual(color["header"], color["camera_info"]["header"])
        self.assertEqual(color["receipt_unix_ns"], 10)
        self.assertEqual(color["camera_info"]["receipt_monotonic_ns"], 21)
        np.testing.assert_array_equal(cv2.imread(str(self.output / color["file"]))[0, 0], [30, 20, 10])
        depth = cv2.imread(str(self.output / saved["streams"]["depth"][0]["file"]), cv2.IMREAD_UNCHANGED)
        self.assertEqual(depth.dtype, np.uint16)
        self.assertTrue(np.all(depth == 1234))
        np.testing.assert_allclose(static_transform(saved["tf_static"], "depth", "color")[:3, 3], [-.02, 0, 0])

    def test_exclusive_output(self):
        self.writer()
        with self.assertRaises(FileExistsError):
            self.writer()

    def test_missing_chain_rejected(self):
        writer = self.writer()
        self.fill(writer)
        with self.assertRaisesRegex(CalibrationError, "Missing static TF chain"):
            writer.finish()
        self.assertFalse((self.output / "snapshot.json").exists())

    def test_pairer_exact_headers_no_info_reuse(self):
        writer = self.writer(frames=2)
        pairer = SnapshotPairer(writer)
        msg, info = messages(np.zeros((4, 6, 3), np.uint8), "color")
        pairer.add("color", msg)
        wrong = messages(np.zeros((4, 6, 3), np.uint8), "color", 2)[1]
        pairer.add("color_info", wrong)
        self.assertEqual(len(writer.record["streams"]["color"]), 0)
        pairer.add("color_info", info)
        pairer.add("color", msg)
        self.assertEqual(len(writer.record["streams"]["color"]), 1)

    def test_profile_change_and_mismatched_info_rejected(self):
        writer = self.writer(frames=2)
        msg, info = messages(np.zeros((4, 6), np.uint16), "depth")
        writer.save("depth", envelope(msg), envelope(info))
        msg, info = messages(np.zeros((4, 6), np.uint16), "depth2", 2)
        with self.assertRaisesRegex(CalibrationError, "profile/frame changed"):
            writer.save("depth", envelope(msg), envelope(info))
        info.header = NS(stamp=NS(sec=3, nanosec=7), frame_id="depth2")
        with self.assertRaisesRegex(CalibrationError, "header must match"):
            writer.save("depth", envelope(msg), envelope(info))

    def test_authorization_and_limits_before_ros(self):
        argv = ["rgbd-snapshot", "--output", str(self.output), "--frames", "1", "--timeout-s", "2",
                "--depth-unit-m", ".001", "--camera-serial", "s", "--profile-record", "p"]
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(argv), 2)
        args = parser().parse_args(argv)
        with self.assertRaisesRegex(CalibrationError, "--subscribe"):
            run_snapshot(args)
        args.subscribe = True
        args.timeout_s = float("inf")
        with self.assertRaisesRegex(CalibrationError, "timeout-s"):
            run_snapshot(args)
        self.assertFalse(self.output.exists())

    def test_import_does_not_load_ros(self):
        code = "import sys; import robot_perception.calibration.rgbd_snapshot; assert not any(n in sys.modules for n in ('rclpy','sensor_msgs','tf2_msgs'))"
        subprocess.run([sys.executable, "-c", code], check=True)

    def test_conflicting_or_cyclic_static_tf_rejected(self):
        writer = self.writer()
        writer.add_tf(tf_message())
        writer.add_tf(tf_message(translation=(.03, 0, 0)))
        with self.assertRaisesRegex(CalibrationError, "Conflicting"):
            static_transform(writer.record["tf_static"], "color", "depth")
        writer.record["tf_static"] = []
        writer.add_tf(tf_message())
        writer.add_tf(tf_message("depth", "color"))
        with self.assertRaisesRegex(CalibrationError, "Cycle"):
            static_transform(writer.record["tf_static"], "color", "depth")
