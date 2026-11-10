import builtins
import contextlib
import io
from importlib.resources import files
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from robot_perception.calibration import calibrate
from robot_perception.calibration.capture import CaptureWriter, Pairer, collect, decode_image, envelope, validate_pair
from robot_perception.calibration.common import CalibrationError, config_errors, make_board, read_json, vision, write_json
from robot_perception.calibration.offline import detect_tags, fit_intrinsics, intrinsics

cv2, np = vision()
EXAMPLE = files("robot_perception.calibration").joinpath("targets.example.json")


def config():
    value = read_json(EXAMPLE)
    value["measurement_record"] = "synthetic test fixture, NOT physical metrology"
    value["board"].update(delivery_confirmed=True, legacy_pattern=True, square_length_m=0.03, marker_length_m=0.022)
    value["apriltags"].update(delivery_confirmed=True, side_lengths_m={"0": 0.03, "1": 0.04})
    value["protocol"].update(approved_by="SYNTHETIC", frozen_on="TEST", record="synthetic test only")
    value["protocol"]["limits"] = {key: 1.0 for key in value["protocol"]["limits"]}
    return value


def draw_board(board):
    if hasattr(board, "generateImage"):
        return board.generateImage((700, 1000))
    return board.draw((700, 1000))


def fixture_images(directory, board):
    """Perspective-render the actual marker board with known camera geometry."""
    image = draw_board(board)
    k = np.array([[820., 0, 640.], [0, 815., 480.], [0, 0, 1.]])
    source = np.float32([[0, 0], [699, 0], [699, 999], [0, 999]])
    board_outer = np.float32([[0, 0, 0], [.21, 0, 0], [.21, .30, 0], [0, .30, 0]])
    names = []
    poses = [(-.4, -.3, -.15), (.35, -.25, .2), (-.3, .4, -.2), (.4, .3, .1),
             (-.1, -.45, .2), (.2, .5, -.1), (-.5, .1, .25), (.45, -.1, -.2),
             (.25, -.35, .15), (-.3, -.2, .1), (.3, .25, -.1), (-.2, .4, .2)]
    for i, pose in enumerate(poses):
        rotation = np.array(pose, dtype=np.float64)
        translation = np.array([-.105 + ((i % 3) - 1) * .07, -.15 + ((i % 4) - 1.5) * .025,
                                .75 + (i % 3) * .08])
        pixels, _ = cv2.projectPoints(board_outer, rotation, translation, k, np.zeros(5))
        h = cv2.getPerspectiveTransform(source, pixels.reshape(4, 2))
        rendered = cv2.warpPerspective(image, h, (1280, 960), flags=cv2.INTER_LINEAR, borderValue=255)
        name = f"image_{i:02d}.png"
        assert cv2.imwrite(str(directory / name), rendered)
        names.append(name)
    return k, names


def fake_header(ns=1_000_000_000, frame="color_optical"):
    return NS(stamp=NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000), frame_id=frame)


def fake_image(ns=1_000_000_000, stream="color"):
    frame = "depth_optical" if stream == "depth" else "color_optical"
    data = np.arange(12, dtype=np.uint8).reshape(2, 2, 3) if stream == "color" else np.array([[0, 255], [256, 65535]], dtype=np.uint16)
    return NS(header=fake_header(ns, frame), width=2, height=2, encoding="rgb8" if stream == "color" else "16UC1",
              step=6 if stream == "color" else 4, is_bigendian=False, data=data.tobytes())


def fake_info(ns=1_000_000_000, stream="color"):
    return NS(header=fake_header(ns, "depth_optical" if stream == "depth" else "color_optical"),
              width=2, height=2, distortion_model="plumb_bob", k=[2., 0, 1, 0, 2., 1, 0, 0, 1],
              d=[0.] * 5, r=np.eye(3).reshape(-1).tolist(), p=[2., 0, 1, 0, 0, 2., 1, 0, 0, 0, 1, 0],
              binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))


def add_frame(pairer, ns=1_000_000_000, offset=0):
    for key in ("color", "depth", "aligned_depth"):
        stamp = ns if key == "color" else ns + offset
        pairer.add(key, fake_image(stamp, key))
        pairer.add(key + "_info", fake_info(stamp, key))


class ConfigTests(unittest.TestCase):
    def test_example_fails_closed(self):
        errors = config_errors(read_json(EXAMPLE), protocol=True)
        self.assertGreater(len(errors), 12)
        self.assertTrue(any("legacy_pattern" in error for error in errors))
        self.assertTrue(any("square_length_m" in error for error in errors))

    def test_invalid_shapes_sizes_ids_and_patterns(self):
        self.assertEqual(config_errors(config(), protocol=True), [])
        cases = [("square_length_m", float("nan")), ("marker_length_m", .05),
                 ("marker_ids", [0] * 35), ("marker_ids", list(range(35)) + [35]),
                 ("squares_x", True), ("legacy_pattern", None), ("delivery_confirmed", False),
                 ("dictionary", "DICT_4X4_50")]
        for key, val in cases:
            with self.subTest(key=key, val=val):
                cfg = config()
                cfg["board"][key] = val
                self.assertTrue(config_errors(cfg, "board"))
        cfg = config()
        cfg["apriltags"]["side_lengths_m"] = {"00": .03, "1": float("inf")}
        self.assertTrue(config_errors(cfg, "tags"))

    def test_nonzero_ids_real_detection_and_parity(self):
        from robot_perception.calibration.common import detect_board, detect_markers
        board, dictionary = make_board(config())
        image = cv2.copyMakeBorder(draw_board(board), 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
        _, marker_ids, _ = detect_markers(image, dictionary)
        self.assertEqual(set(marker_ids.reshape(-1)), set(range(100, 135)))
        corners, ids = detect_board(image, board, dictionary)
        self.assertEqual(len(ids), 54)
        wrong = config()
        wrong["board"]["legacy_pattern"] = False
        if not hasattr(board, "setLegacyPattern"):
            with self.assertRaisesRegex(CalibrationError, "cannot represent"):
                make_board(wrong)
        else:
            wrong_board, _ = make_board(wrong)
            wrong_image = draw_board(wrong_board)
            self.assertFalse(np.array_equal(draw_board(board), wrong_image))

    def test_json_exclusive_and_finite(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.json"
            write_json(path, {"x": 1})
            with self.assertRaises(FileExistsError):
                write_json(path, {"x": 2})
            self.assertEqual(read_json(path), {"x": 1})
            with self.assertRaises(ValueError):
                write_json(Path(tmp) / "nan.json", {"x": float("nan")})
            self.assertFalse((Path(tmp) / "nan.json").exists())


class OfflineTests(unittest.TestCase):
    def test_rendered_tag_detection_overlay_and_no_overwrite(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
        canvas = np.full((400, 700), 255, dtype=np.uint8)
        canvas[50:250, 50:250] = draw(dictionary, 0, 200)
        canvas[50:250, 350:550] = draw(dictionary, 1, 200)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root / "config.json", config())
            cv2.imwrite(str(root / "tags.png"), canvas)
            result = detect_tags(root / "config.json", config(), root / "tags.png", root / "out")
            self.assertEqual({r["id"] for r in result["detections"]}, {0, 1})
            self.assertEqual(result["status"], "detection_only_not_pose")
            self.assertEqual(result["missing_selected_ids"], [])
            self.assertTrue((root / "out/overlay.png").exists())
            with self.assertRaises(CalibrationError):
                detect_tags(root / "config.json", config(), root / "tags.png", root / "out")

    def test_intrinsics_rendered_board_known_k_heldout_and_manifest_guards(self):
        board, _ = make_board(config())
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            k, names = fixture_images(root, board)
            write_json(root / "config.json", config())
            manifest = {"image_geometry": "raw_color_no_crop_no_resize", "candidate_model": "opencv_brown5",
                        "camera_serial": "SYNTHETIC", "split_frozen_on": "BEFORE_FIT_SYNTHETIC",
                        "image_size": [1280, 960], "train": names[:9], "heldout": names[9:]}
            write_json(root / "manifest.json", manifest)
            result = intrinsics(root / "config.json", config(), root / "manifest.json", root / "candidate.json")
            estimated = np.array(result["k"])
            np.testing.assert_allclose(estimated[:2, :], k[:2, :], atol=12)
            self.assertLess(result["heldout_summary"]["pooled_corner_rms_px"], .6)
            self.assertEqual(result["heldout_summary"]["successful"], 3)
            self.assertEqual(result["status"], "candidate_not_approved")
            with self.assertRaisesRegex(CalibrationError, "existing output"):
                intrinsics(root / "config.json", config(), root / "manifest.json", root / "candidate.json")
            manifest["heldout"][0] = manifest["train"][0]
            write_json(root / "duplicate.json", manifest)
            with self.assertRaisesRegex(CalibrationError, "Duplicate"):
                intrinsics(root / "config.json", config(), root / "duplicate.json", root / "no.json")
            manifest["heldout"][0] = names[9]
            manifest["image_size"] = [640, 480]
            write_json(root / "wrong_size.json", manifest)
            with self.assertRaisesRegex(CalibrationError, "size changed"):
                intrinsics(root / "config.json", config(), root / "wrong_size.json", root / "no.json")
            manifest["image_size"] = [1280, 960]
            # A unique blank heldout must be reported, not silently filtered.
            cv2.imwrite(str(root / names[9]), np.full((960, 1280), 240, dtype=np.uint8))
            write_json(root / "missing.json", manifest)
            with self.assertRaisesRegex(CalibrationError, "Too few usable"):
                intrinsics(root / "config.json", config(), root / "missing.json", root / "failed.json")
            failed = read_json(root / "failed.json")
            self.assertEqual(failed["views"]["heldout"][0]["status"], "insufficient_corners")
            self.assertEqual(failed["status"], "not_solved")

    def test_degenerate_views_rejected(self):
        from robot_perception.calibration.common import board_points
        board, _ = make_board(config())
        obj = board_points(board)
        k = np.array([[800., 0, 640], [0, 800., 480], [0, 0, 1]])
        img, _ = cv2.projectPoints(obj, np.zeros(3), np.array([-.1, -.15, .8]), k, np.zeros(5))
        with self.assertRaisesRegex(CalibrationError, "diversity"):
            fit_intrinsics([obj] * 6, [img] * 6, (1280, 960))


class CaptureTests(unittest.TestCase):
    def test_gate_before_ros_import_or_config_access(self):
        original_import = builtins.__import__

        def guarded(name, *args, **kwargs):
            if name.startswith(("rclpy", "sensor_msgs", "tianji", "pyrealsense")):
                self.fail(f"Hardware-related import attempted: {name}")
            return original_import(name, *args, **kwargs)

        args = ["capture", "--config", "/missing_config", "--target", "board", "--output", "/not_created",
                "--samples", "1", "--timeout-s", "1", "--interval-s", "1", "--max-pair-delta-ms", "1", "--depth-unit-m", ".001",
                "--camera-serial", "123456789012", "--profile-record", "synthetic fixture only"]
        with patch("builtins.__import__", guarded), contextlib.redirect_stderr(io.StringIO()) as stderr:
            self.assertEqual(calibrate.main(args), 2)
            self.assertIn("--subscribe", stderr.getvalue())
            args[args.index("/missing_config")] = str(EXAMPLE)
            self.assertEqual(calibrate.main(args + ["--subscribe"]), 2)
            self.assertIn("Configuration blocked", stderr.getvalue())
            args[args.index("--timeout-s") + 1] = "nan"
            self.assertEqual(calibrate.main(args + ["--subscribe"]), 2)

    def test_pairing_bounded_exact_info_and_no_reuse(self):
        pairer = Pairer(2, queue_size=2)
        add_frame(pairer, offset=1_000_000)
        selected = pairer.pop()
        self.assertEqual(len(selected), 6)
        self.assertIsNone(pairer.pop())
        add_frame(pairer)  # Already used timestamps are never accepted again.
        self.assertIsNone(pairer.pop())
        for i in range(5):
            pairer.add("color", fake_image(2_000_000_000 + i))
        self.assertEqual(len(pairer.queues["color"]), 2)
        self.assertGreater(pairer.dropped["color"], 0)
        pairer = Pairer(1)
        add_frame(pairer, offset=2_000_000)
        self.assertIsNone(pairer.pop())
        pairer = Pairer(1)
        add_frame(pairer)
        pairer.queues["color_info"][0]["header"]["frame_id"] = "wrong"
        self.assertIsNone(pairer.pop())

    def test_decode_uint16_padding_and_endian(self):
        msg = fake_image(stream="depth")
        expected = np.array([[0, 255], [256, 65535]], dtype=np.uint16)
        np.testing.assert_array_equal(decode_image(msg, "depth"), expected)
        rows = np.zeros((2, 6), dtype=np.uint8)
        rows[:, :4] = expected.astype(">u2").view(np.uint8).reshape(2, 4)
        msg.step, msg.is_bigendian, msg.data = 6, True, rows.tobytes()
        np.testing.assert_array_equal(decode_image(msg, "depth"), expected)
        msg.encoding = "32FC1"
        with self.assertRaises(CalibrationError):
            decode_image(msg, "depth")

    def test_writer_lossless_full_metadata_and_profile_checks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "session"
            acquisition = {"camera_serial": "123456789012", "profile_record": "synthetic",
                           "device_identity_source": "operator_confirmed_not_verified_by_subscriber",
                           "samples_requested": 2, "timeout_s": 1., "interval_s": .1}
            writer = CaptureWriter(root, {"config": config()}, 2, .001, acquisition)
            self.assertEqual(read_json(root / "session.json")["acquisition"], acquisition)
            pairer = Pairer(2)
            add_frame(pairer, offset=1_000_000)
            writer.save(pairer.pop())
            sample = read_json(root / "000000/sample.json")
            self.assertEqual(sample["pair_delta_ms"]["max_span"], 1.)
            self.assertIn("receipt_monotonic_ns", sample["streams"]["color"]["camera_info"])
            restored = cv2.imread(str(root / "000000/depth.png"), cv2.IMREAD_UNCHANGED)
            np.testing.assert_array_equal(restored, np.array([[0, 255], [256, 65535]], dtype=np.uint16))
            self.assertEqual(restored.dtype, np.uint16)
            with self.assertRaises(FileExistsError):
                CaptureWriter(root, {}, 2, .001, acquisition)
            add_frame(pairer, ns=2_000_000_000)
            selected = pairer.pop()
            selected["color_info"]["message"].k[0] = float("nan")
            with self.assertRaisesRegex(CalibrationError, "Non-finite"):
                writer.save(selected)
            self.assertFalse((root / "000001").exists())
            selected["color_info"]["message"].k[0] = 3.
            with self.assertRaisesRegex(CalibrationError, "profile changed"):
                writer.save(selected)

    def test_bad_info_rejected(self):
        image, info = envelope(fake_image()), envelope(fake_info())
        info["message"].width = 3
        with self.assertRaisesRegex(CalibrationError, "dimensions"):
            validate_pair(image, info)
        info["message"].width = 2
        info["message"].roi.width = 2
        with self.assertRaisesRegex(CalibrationError, "ROI"):
            validate_pair(image, info)

    def test_finite_loop_timeout_and_sample_count(self):
        now = [0.]
        pairer = Pairer(1)
        writer = NS(count=0)

        def save(items):
            writer.count += 1

        writer.save = save

        def spin_empty(timeout):
            now[0] += timeout

        self.assertFalse(collect(pairer, writer, 2, .5, .1, spin_empty, clock=lambda: now[0]))
        self.assertLessEqual(now[0], .6)
        self.assertEqual(writer.count, 0)

        def spin_frames(timeout):
            now[0] += timeout
            add_frame(pairer, ns=1_000_000_000 + int(now[0] * 1e9))

        self.assertTrue(collect(pairer, writer, 2, 1., .1, spin_frames, clock=lambda: now[0]))
        self.assertEqual(writer.count, 2)


if __name__ == "__main__":
    unittest.main()
