"""Offline geometry/session/live matching tests; never connect hardware or ROS."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

from robot_perception.calibration.common import CalibrationError
from robot_perception.calibration.extrinsics import (camera_info, defaults, detect, digest, excitation, inverse, object_points,
                        planar_candidates, project, se3, solve, transform, validate_config,
                        validate_reference)
from robot_perception.calibration.extrinsics_live import RGBMatcher
from robot_perception.calibration.extrinsics_session import Session, load_offline, validate_packet


def config(mode="known_board"):
    c = defaults()
    c.update(mode=mode, fixed_camera=True, raw_model_confirmed=True, profile_record="synthetic camera profile")
    c["board"].update(scale_confirmed=True, measurement_record="synthetic 0.020m exact scale")
    c["thresholds"] = {"translation_mm": .1, "rotation_deg": .01, "reprojection_px": .05,
                       "confirmed": True, "record": "synthetic test limits, not a deployment protocol"}
    return c


def info(index=0, now=False):
    stamp = time.time_ns() if now else 1700000000000000000 + index * 1000000000
    k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    return {"width": 640, "height": 480, "distortion_model": "plumb_bob", "k": k, "d": [0.] * 5,
            "r": np.eye(3).ravel().tolist(), "p": np.c_[np.array(k).reshape(3, 3), np.zeros(3)].ravel().tolist(),
            "header": {"sec": stamp // 10**9, "nanosec": stamp % 10**9, "frame_id": "d435_color_optical_frame"},
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}


def packet(image, ci=None):
    ci = ci or info()
    return {"image": image, "camera_info": ci, "image_header": copy.deepcopy(ci["header"]),
            "image_geometry": "raw_color_no_crop_no_resize", "source": "offline",
            "provenance": {"synthetic": "unit test, not hardware"}}


def fixture(mode="known_board", n=None):
    cfg = config(mode)
    session = Session(cfg)
    x = transform([.2, -.13, .31], [.4, -.3, .2])
    y = transform([-.12, .23, .09], [.03, .01, .07])
    rng = np.random.default_rng(42)
    n = n or (5 if mode == "known_board" else 10)
    train = 3 if mode == "known_board" else 7
    for i in range(n):
        c = transform(rng.uniform(-.5, .5, 3), [-.11 + rng.uniform(-.04, .04),
                                               -.08 + rng.uniform(-.03, .03), .9 + rng.uniform(-.1, .1)])
        ci = info(i)
        k, d = camera_info(ci)
        corners = project(c, object_points(cfg["board"]), k, d)
        candidates = planar_candidates(corners, ci, cfg["board"])
        selected = min(range(len(candidates)), key=lambda j: np.linalg.norm(np.array(candidates[j]["T_C_Q"]) - c))
        image = np.full((480, 640, 3), i * 10, np.uint8)
        det = {"corners": corners.tolist(), "candidates": candidates, "reverse": False, "status": "synthetic"}
        session.add(packet(image, ci), det)
        a = x @ c if mode == "known_board" else x @ c @ inverse(y)
        ref = {"units": "m", "from_frame": "board" if mode == "known_board" else "end_effector",
               "to_frame": "base", "matrix": a.tolist(), "source_record": f"synthetic independent pose {i}",
               "settled_same_pose": True}
        session.update(i, selected=selected, candidate_confirmed=True, orientation_confirmed=True,
                       reference=ref, split="train" if i < train else "heldout")
    return session, x, y


def board_image(cfg=None):
    cfg = cfg or config()
    # Render full checkerboard. Printed pattern starts one square before first inner corner.
    b = cfg["board"]
    side = 60
    pattern = np.empty((b["squares_y"] * side, b["squares_x"] * side), np.uint8)
    for y in range(b["squares_y"]):
        for x in range(b["squares_x"]):
            pattern[y * side:(y + 1) * side, x * side:(x + 1) * side] = 0 if (x + y) % 2 == 0 else 255
    c = transform([.25, -.32, .08], [-.1, -.07, .65])
    s = b["square_m"]
    bounds = np.array([[-s, -s, 0], [(b["squares_x"] - 1)*s, -s, 0],
                       [(b["squares_x"] - 1)*s, (b["squares_y"] - 1)*s, 0],
                       [-s, (b["squares_y"] - 1)*s, 0]])
    k, d = camera_info(info())
    dst = project(c, bounds, k, d).astype(np.float32)
    src = np.array([[0, 0], [pattern.shape[1], 0], [pattern.shape[1], pattern.shape[0]], [0, pattern.shape[0]]], np.float32)
    h = cv2.getPerspectiveTransform(src, dst)
    gray = cv2.warpPerspective(pattern, h, (640, 480), borderValue=180)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


class GeometryTests(unittest.TestCase):
    def test_known_board_direction_and_heldout(self):
        session, x, _ = fixture()
        result = session.run_solve()
        np.testing.assert_allclose(result["T_B_C"], x, atol=1e-8)
        self.assertIsNone(result["T_E_Q"])
        self.assertTrue(result["threshold_pass"])
        self.assertEqual([m["split"] for m in result["metrics"]], ["train"] * 3 + ["heldout"] * 2)
        self.assertEqual(result["status"], "candidate_not_approved")

    def test_handeye_mapping_and_heldout(self):
        session, x, y = fixture("eye_to_hand")
        result = session.run_solve()
        np.testing.assert_allclose(result["T_B_C"], x, atol=1e-8)
        np.testing.assert_allclose(result["T_E_Q"], y, atol=1e-8)
        self.assertTrue(result["threshold_pass"])

    def test_heldout_never_fits_either_mode(self):
        for mode in ("known_board", "eye_to_hand"):
            session, _, _ = fixture(mode)
            first = session.run_solve()
            heldout = next(i for i, s in enumerate(session.samples) if s["split"] == "heldout")
            ref = copy.deepcopy(session.samples[heldout]["reference"])
            ref["matrix"][0][3] += .05
            session.update(heldout, reference=ref)
            second = session.run_solve()
            np.testing.assert_allclose(first["T_B_C"], second["T_B_C"], atol=1e-12)
            if mode == "eye_to_hand":
                np.testing.assert_allclose(first["T_E_Q"], second["T_E_Q"], atol=1e-12)
            self.assertFalse(second["threshold_pass"])
            self.assertEqual(second["status"], "failed_diagnostic_not_approved")
            self.assertTrue(next(m for m in second["metrics"] if m["id"] == session.samples[heldout]["id"])["failures"])

    def test_degenerate_rejection(self):
        session, _, _ = fixture("eye_to_hand")
        session.samples[1]["reference"]["matrix"] = session.samples[0]["reference"]["matrix"]
        with self.assertRaisesRegex(CalibrationError, "Duplicate"):
            session.run_solve()
        for axis in ([0, 0, 0], [0, 0, 1]):
            poses = [transform(np.array(axis) * i * .1, [i * .01, (i % 2) * .02, .1]) for i in range(7)]
            with self.assertRaisesRegex(CalibrationError, "rotational"):
                excitation(poses)
        poses = [transform([i*.1, (i % 2)*.15, 0], [0, 0, .1]) for i in range(7)]
        with self.assertRaisesRegex(CalibrationError, "translation"):
            excitation(poses)

    def test_se3_rejects_invalid(self):
        invalid = [np.zeros((3, 3)), np.diag([1, 1, -1, 1]), np.diag([2, 1, 1, 1])]
        bottom = np.eye(4)
        bottom[3, 0] = .1
        invalid.append(bottom)
        nan = np.eye(4)
        nan[0, 3] = np.nan
        invalid.append(nan)
        for matrix in invalid:
            with self.assertRaises(CalibrationError):
                se3(matrix)

    def test_mandatory_confirmations_counts_and_reference(self):
        session, _, _ = fixture()
        for key in ("fixed_camera", "raw_model_confirmed"):
            c = copy.deepcopy(session.config)
            c[key] = False
            with self.assertRaises(CalibrationError):
                solve(c, session.samples)
        for field, value in (("scale_confirmed", False), ("measurement_record", ""), ("square_m", float("nan"))):
            c = copy.deepcopy(session.config)
            c["board"][field] = value
            with self.assertRaises(CalibrationError):
                solve(c, session.samples)
        c = copy.deepcopy(session.config)
        c["thresholds"]["confirmed"] = False
        with self.assertRaises(CalibrationError):
            solve(c, session.samples)
        for field, value in (("reference", None), ("orientation_confirmed", False), ("candidate_confirmed", False)):
            samples = copy.deepcopy(session.samples)
            samples[0][field] = value
            with self.assertRaises(CalibrationError):
                solve(session.config, samples)
        with self.assertRaisesRegex(CalibrationError, "Need"):
            solve(session.config, session.samples[:-1])
        ref = copy.deepcopy(session.samples[0]["reference"])
        for field, value in (("units", "mm"), ("from_frame", "base"), ("source_record", ""), ("settled_same_pose", False)):
            with self.assertRaises(CalibrationError):
                validate_reference({**ref, field: value}, session.config)

    def test_camera_model_validation(self):
        camera_info(info())
        changes = [{"distortion_model": "equidistant"}, {"d": [0.] * 4}, {"width": 0},
                   {"binning_x": 2}, {"binning_x": None}, {"roi": {}},
                   {"r": transform([.1, 0, 0], [0, 0, 0])[:3, :3].ravel().tolist()},
                   {"k": [float("inf")] * 9}, {"p": [0.] * 12}]
        for change in changes:
            with self.assertRaises(CalibrationError):
                camera_info({**info(), **change})
        p = packet(np.zeros((400, 640, 3), np.uint8))
        with self.assertRaisesRegex(CalibrationError, "size"):
            validate_packet(p, config())

    def test_checkerboard_planar_candidates_and_reverse(self):
        cfg = config()
        image = board_image(cfg)
        normal = detect(image, info(), cfg["board"], raw_model_confirmed=True)
        reverse = detect(image, info(), cfg["board"], True, raw_model_confirmed=True)
        self.assertEqual(len(normal["corners"]), 88)
        self.assertEqual(len(normal["candidates"]), 2)
        # Independent SB calls may vary by subpixel numerical optimisation.
        np.testing.assert_allclose(normal["corners"], reverse["corners"][::-1], atol=.02)
        self.assertTrue(all(c["positive_depth"] for c in normal["candidates"]))
        self.assertLess(min(c["rms_px"] for c in normal["candidates"]), 1.)
        self.assertGreater(np.linalg.norm(np.array(normal["candidates"][0]["T_C_Q"]) - reverse["candidates"][0]["T_C_Q"]), 1.)
        session = Session(cfg)
        session.add(packet(image), normal)
        self.assertIsNone(session.samples[0]["selected"])
        self.assertFalse(session.samples[0]["candidate_confirmed"])
        session.reverse_order(0)
        self.assertTrue(session.samples[0]["reverse"])
        self.assertIsNone(session.samples[0]["reference"])


class SessionTests(unittest.TestCase):
    def test_incomplete_solve_does_not_freeze_capture_or_split(self):
        full, _, _ = fixture()
        incomplete = Session(config())
        with self.assertRaisesRegex(CalibrationError, "Need"):
            incomplete.run_solve()
        self.assertIsNone(incomplete.split_freeze)
        sample = full.samples[0]
        image = cv2.imdecode(np.frombuffer(full.images[sample["id"]], np.uint8), cv2.IMREAD_COLOR)
        incomplete.add({**copy.deepcopy(sample["metadata"]), "image": image}, sample)
        incomplete.update(0, split="heldout")
        self.assertEqual(incomplete.samples[0]["split"], "heldout")
        self.assertIsNone(incomplete.split_freeze)

        full.update(0, reference=None)
        with self.assertRaisesRegex(CalibrationError, "reference"):
            full.run_solve()
        self.assertIsNone(full.split_freeze)
        full.update(0, split="heldout")
        self.assertEqual(full.samples[0]["split"], "heldout")

    def test_split_frozen_before_first_fit(self):
        session, _, _ = fixture()
        session.run_solve()
        self.assertEqual(session.split_freeze["assignments"]["000000"], "train")
        with self.assertRaisesRegex(CalibrationError, "Split frozen"):
            session.update(0, split="heldout")
        session.samples[0]["split"] = "heldout"
        with self.assertRaisesRegex(CalibrationError, "Frozen split"):
            session.validate()

    def test_unconfirmed_scale_and_model_only_detect_corners(self):
        cfg = defaults()
        image = board_image()
        detection = detect(image, info(), cfg["board"])
        self.assertEqual(len(detection["corners"]), 88)
        self.assertEqual(detection["candidates"], [])
        Session(cfg).add(packet(image), detection)
        with self.assertRaisesRegex(CalibrationError, "scale"):
            planar_candidates(detection["corners"], info(), cfg["board"])
        cfg = config()
        detection = detect(image, info(), cfg["board"], raw_model_confirmed=False)
        self.assertEqual(detection["candidates"], [])

    def test_save_open_export_no_overwrite(self):
        session, _, _ = fixture()
        session.run_solve()
        with tempfile.TemporaryDirectory(prefix="extrinsics-test-", dir="/tmp") as tmp:
            directory = Path(tmp) / "session"
            session.save(directory)
            reopened = Session.open(directory)
            self.assertEqual(digest(reopened.inputs()), digest(session.inputs()))
            self.assertIsNone(reopened.result)
            with self.assertRaises(CalibrationError):
                reopened.export(Path(tmp) / "stale.json")
            with self.assertRaises(CalibrationError):
                session.save(directory)
            output = Path(tmp) / "candidate.json"
            session.export(output)
            with self.assertRaises(FileExistsError):
                session.export(output)
            raw = json.loads(output.read_text())
            self.assertEqual(raw["input_sha256"], digest(session.inputs()))
            self.assertEqual(len(raw["inputs"]["samples"]), 5)

    def test_corrupt_session_and_png(self):
        session, _, _ = fixture()
        with tempfile.TemporaryDirectory(prefix="extrinsics-test-", dir="/tmp") as tmp:
            directory = Path(tmp) / "s"
            session.save(directory)
            p = directory / "session.json"
            raw = json.loads(p.read_text())
            raw["samples"][0]["candidates"][0]["T_C_Q"][0][3] += 1
            # Even a recomputed input hash cannot bypass geometric revalidation.
            raw["input_sha256"] = digest({k: v for k, v in raw.items() if k != "input_sha256"})
            p.write_text(json.dumps(raw))
            with self.assertRaisesRegex(CalibrationError, "Corrupt IPPE"):
                Session.open(directory)
            p.write_text(json.dumps({**session.inputs(), "input_sha256": digest(session.inputs())}))
            (directory / "000000.png").write_bytes(b"not a PNG")
            with self.assertRaisesRegex(CalibrationError, "hash"):
                Session.open(directory)

    def test_invalidation_and_guarded_configuration(self):
        session, _, _ = fixture()
        session.run_solve()
        session.update(0, excluded=True, exclusion_reason="test rejection")
        self.assertIsNone(session.result)
        with self.assertRaises(CalibrationError):
            session.update(1, excluded=True, exclusion_reason="")
        c = copy.deepcopy(session.config)
        c["frames"]["base"] = "other"
        with self.assertRaisesRegex(CalibrationError, "new session"):
            session.configure(c)
        c = copy.deepcopy(session.config)
        c["board"]["square_m"] = .03
        with self.assertRaises(CalibrationError):
            session.configure(c)
        session.update(0, excluded=False)
        session.run_solve()
        session.config["thresholds"]["translation_mm"] = 9
        with tempfile.TemporaryDirectory(prefix="extrinsics-test-", dir="/tmp") as tmp:
            with self.assertRaisesRegex(CalibrationError, "Stale"):
                session.export(Path(tmp) / "stale.json")

    def test_profile_duplicates_and_reference_gate(self):
        s = Session(config())
        p = packet(board_image())
        detection = detect(p["image"], p["camera_info"], s.config["board"], raw_model_confirmed=True)
        s.add(p, detection)
        with self.assertRaisesRegex(CalibrationError, "Duplicate"):
            s.add(p, detection)
        p2 = copy.deepcopy(p)
        p2["image"][0, 0] = 0
        p2["camera_info"]["d"][0] = .01
        with self.assertRaisesRegex(CalibrationError, "profile"):
            s.add(p2, detection)
        with self.assertRaisesRegex(CalibrationError, "BEFORE"):
            s.update(0, reference={"units": "m"})

    def test_offline_sidecar_and_diagnostic_import(self):
        with tempfile.TemporaryDirectory(prefix="extrinsics-test-", dir="/tmp") as tmp:
            p = Path(tmp)
            image = board_image()
            cv2.imwrite(str(p / "color_00.png"), image)
            ci = info()
            sidecar = {k: v for k, v in packet(image, ci).items() if k != "image"}
            (p / "info.json").write_text(json.dumps(sidecar))
            imported = load_offline(p / "color_00.png", p / "info.json")
            validate_packet(imported, config())
            ci = {k: v for k, v in ci.items() if k not in ("roi", "binning_x", "binning_y", "header")}
            snapshots = {"snapshots": [{"stream": "color", "file": "color_00.png", "camera_info": ci,
                                        "header": info()["header"]}]}
            (p / "snapshots.json").write_text(json.dumps(snapshots))
            with self.assertRaisesRegex(CalibrationError, "explicitly"):
                load_offline(p / "color_00.png", p / "snapshots.json")
            imported = load_offline(p / "color_00.png", p / "snapshots.json", True)
            validate_packet(imported, config())
            self.assertTrue(imported["provenance"]["omitted_roi_binning_raw_confirmed_by_operator"])


class LiveTests(unittest.TestCase):
    def test_exact_bounded_latest_matching(self):
        matcher = RGBMatcher(capacity=2)
        image = np.zeros((480, 640, 3), np.uint8)
        for i in range(3):
            matcher.add_image(image, info(i)["header"])
        self.assertEqual(len(matcher.images), 2)
        matcher.add_info(info(0))
        self.assertIsNone(matcher.pop())
        matcher.add_info(info(2))
        p = matcher.pop()
        self.assertEqual(p["image_header"], info(2)["header"])
        self.assertIsNone(matcher.pop())
        matcher.add_image(image, info(2)["header"])
        matcher.add_info(info(2))
        self.assertIsNone(matcher.pop())
        changed = info(3)
        changed["d"][0] = .02
        with self.assertRaisesRegex(CalibrationError, "profile"):
            matcher.add_info(changed)

    def test_frame_dimensions_freshness(self):
        matcher = RGBMatcher()
        ci = info(now=True)
        matcher.add_image(np.zeros((480, 640, 3), np.uint8), ci["header"])
        other = copy.deepcopy(ci)
        other["header"]["frame_id"] = "wrong"
        matcher.add_info(other)
        self.assertIsNone(matcher.pop())
        matcher = RGBMatcher()
        matcher.add_image(np.zeros((400, 640, 3), np.uint8), ci["header"])
        matcher.add_info(ci)
        with self.assertRaisesRegex(CalibrationError, "dimensions"):
            matcher.pop()
        matcher = RGBMatcher()
        matcher.add_image(np.zeros((480, 640, 3), np.uint8), ci["header"])
        matcher.add_info(ci)
        p = matcher.pop()
        validate_packet(p, config(), 2)
        p["image_receipt_monotonic_ns"] -= 10**10
        with self.assertRaisesRegex(CalibrationError, "Stale"):
            validate_packet(p, config(), 2)
        p["image_receipt_monotonic_ns"] = time.monotonic_ns()
        p["camera_info"]["header"]["sec"] -= 10
        p["image_header"] = p["camera_info"]["header"]
        with self.assertRaisesRegex(CalibrationError, "Stale source"):
            validate_packet(p, config(), 2)


class QtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def wait_solver(self, window):
        deadline = time.monotonic() + 10
        while window.solver is not None and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(.01)
        self.assertIsNone(window.solver)

    def test_offline_buttons_and_workers_close(self):
        from robot_perception.calibration.extrinsics_ui import Window
        from PyQt5 import QtWidgets as W
        window = Window(Session(config()))
        try:
            window.show()
            with tempfile.TemporaryDirectory(prefix="extrinsics-qt-", dir="/tmp") as tmp:
                directory = Path(tmp)
                image = board_image()
                image_path = directory / "color.png"
                info_path = directory / "info.json"
                cv2.imwrite(str(image_path), image)
                info_path.write_text(json.dumps({k: v for k, v in packet(image).items() if k != "image"}))
                with patch.object(W.QFileDialog, "getOpenFileName", side_effect=[(str(image_path), ""), (str(info_path), "")]):
                    window.import_button.click()
                deadline = time.monotonic() + 10
                while window.detection is None and time.monotonic() < deadline:
                    self.app.processEvents()
                    time.sleep(.02)
                self.assertIsNotNone(window.detection)
                self.assertEqual(len(window.detection["corners"]), 88)
                window.capture_button.click()
                self.assertEqual(len(window.session.samples), 1)
                self.assertIsNone(window.session.samples[0]["reference"])
                self.assertIsNone(window.candidate.currentData())
                window.reverse_button.click()
                self.assertTrue(window.session.samples[0]["reverse"])
                window.split.setCurrentText("heldout")
                window.candidate.setCurrentIndex(1)
                window.orientation_confirm.setChecked(True)
                window.candidate_confirm.setChecked(True)
                ref = {"units": "m", "from_frame": "board", "to_frame": "base", "matrix": np.eye(4).tolist(),
                       "source_record": "explicit synthetic button-test reference", "settled_same_pose": True}
                ref_path = directory / "ref.json"
                ref_path.write_text(json.dumps(ref))
                with patch.object(W.QFileDialog, "getOpenFileName", return_value=(str(ref_path), "")):
                    window.reference_button.click()
                window.apply_button.click()
                self.assertEqual(window.session.samples[0]["split"], "heldout")
                self.assertEqual(window.session.samples[0]["reference"], ref)
                with patch.object(W.QFileDialog, "getSaveFileName", return_value=(str(directory / "session"), "")):
                    window.save_button.click()
                self.assertTrue((directory / "session" / "session.json").exists())
                with patch.object(W.QFileDialog, "getExistingDirectory", return_value=str(directory / "session")), \
                        patch.object(W.QMessageBox, "question", return_value=W.QMessageBox.Yes):
                    window.open_button.click()
                self.assertIsNone(window.session.result)
                self.assertFalse(window.export_button.isEnabled())
                self.assertIsNone(window.live)
        finally:
            window.close()
            self.app.processEvents()
        self.assertFalse(window.detector.is_alive())

    @unittest.skipUnless(os.environ.get("ROBOT_PERCEPTION_FIXTURE_DIR"),
                         "Set ROBOT_PERCEPTION_FIXTURE_DIR to external read-only vision_calibration data")
    def test_local_real_diagnostic_import_preview_only(self):
        from robot_perception.calibration.extrinsics_ui import Window
        base = Path(os.environ["ROBOT_PERCEPTION_FIXTURE_DIR"]) / "board_diagnostic_20261109_210647"
        window = Window()
        try:
            window.snapshot_confirm.setChecked(True)
            for index in range(3):
                window.import_paths(base / f"color_{index:02d}.png", base / "snapshots.json")
                deadline = time.monotonic() + 15
                while window.detection is None and time.monotonic() < deadline:
                    self.app.processEvents()
                    time.sleep(.02)
                self.assertIsNotNone(window.detection)
                self.assertEqual(len(window.detection["corners"]), 88)
                self.assertEqual(window.detection["candidates"], [])
                window.capture_button.click()
            self.assertEqual(len(window.session.samples), 3)
            self.assertTrue(all(s["reference"] is None for s in window.session.samples))
            self.assertIsNone(window.session.result)
        finally:
            window.close()
            self.app.processEvents()

    def test_eye_to_hand_offscreen_solve_export(self):
        from robot_perception.calibration.extrinsics_ui import Window
        from PyQt5 import QtWidgets as W
        session, x, y = fixture("eye_to_hand")
        window = Window(session)
        try:
            window.solve_button.click()
            self.wait_solver(window)
            self.assertTrue(window.export_button.isEnabled())
            np.testing.assert_allclose(window.session.result["T_B_C"], x, atol=1e-8)
            np.testing.assert_allclose(window.session.result["T_E_Q"], y, atol=1e-8)
            with tempfile.TemporaryDirectory(prefix="extrinsics-qt-", dir="/tmp") as tmp:
                path = Path(tmp) / "handeye.json"
                with patch.object(W.QFileDialog, "getSaveFileName", return_value=(str(path), "")):
                    window.export_button.click()
                result = json.loads(path.read_text())
                self.assertTrue(result["threshold_pass"])
                self.assertIsNotNone(result["inputs"]["split_freeze"])
        finally:
            window.close()
            self.app.processEvents()

    def test_async_solve_discards_changed_inputs(self):
        from robot_perception.calibration.extrinsics_ui import Window
        session, _, _ = fixture()
        window = Window(session)
        try:
            window.solve_button.click()
            window.thresholds["rotation_deg"].setText("1.0")
            self.wait_solver(window)
            self.assertIsNone(window.session.result)
            self.assertFalse(window.export_button.isEnabled())
            self.assertIn("过期", window.status.text())
        finally:
            window.close()
            self.app.processEvents()

    def test_solve_export_and_immediate_widget_invalidation(self):
        from robot_perception.calibration.extrinsics_ui import Window
        from PyQt5 import QtWidgets as W
        session, x, _ = fixture()
        window = Window(session)
        try:
            window.solve_button.click()
            self.wait_solver(window)
            self.assertTrue(window.export_button.isEnabled())
            np.testing.assert_allclose(window.session.result["T_B_C"], x, atol=1e-8)
            with tempfile.TemporaryDirectory(prefix="extrinsics-qt-", dir="/tmp") as tmp:
                path = Path(tmp) / "candidate.json"
                with patch.object(W.QFileDialog, "getSaveFileName", return_value=(str(path), "")):
                    window.export_button.click()
                self.assertTrue(path.exists())
            window.thresholds["translation_mm"].setText("2")
            self.assertIsNone(window.session.result)
            self.assertFalse(window.export_button.isEnabled())
            window.solve_button.click()
            self.wait_solver(window)
            self.assertTrue(window.export_button.isEnabled())
            window.table.selectRow(0)
            window.reference.setPlainText("")
            self.assertIsNone(window.session.result)
            self.assertFalse(window.export_button.isEnabled())
        finally:
            window.close()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
