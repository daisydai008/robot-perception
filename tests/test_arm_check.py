"""Synthetic mesh/raster/ICP tests; no ROS, drivers, controllers or network."""
import contextlib
import copy
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import zipfile

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from robot_perception.calibration.arm_check import (
    Group, apply, arm_chains, cross_validate, exp_se3, load_snapshot, load_visuals,
    make_groups, observability, pose_meshes, read_states, read_stl, refine, residual_image, run_arm_check)
from robot_perception.calibration.calibrate import main, parser
from robot_perception.calibration.capture import envelope
from robot_perception.calibration.common import CalibrationError, sha256
from robot_perception.calibration.extrinsics import inverse
from robot_perception.calibration.rgbd_snapshot import SnapshotWriter
from robot_perception.calibration.urdf_fk import load_urdf, origin_transform
from test_rgbd_snapshot import messages, tf_message


def box_triangles(size=(.16, .10, .16)):
    vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * np.array(size) / 2
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    triangles = []
    for a, b, c, d in faces:
        triangles.extend([vertices[[a, b, c]], vertices[[a, c, d]]])
    return np.array(triangles)


def stl_bytes(triangles, binary):
    if binary:
        data = bytearray(b"solid binary STL".ljust(80, b" ") + struct.pack("<I", len(triangles)))
        for triangle in triangles:
            data += struct.pack("<12fH", *([0.] * 3 + triangle.ravel().tolist()), 0)
        return bytes(data)
    lines = ["solid test"]
    for triangle in triangles:
        lines.extend(["facet normal 0 0 0", "outer loop"])
        lines.extend("vertex " + " ".join(map(str, point)) for point in triangle)
        lines.extend(["endloop", "endfacet"])
    return ("\n".join(lines + ["endsolid test"]) + "\n").encode("ascii")


def synthetic_urdf(path):
    xml = ['<robot name="synthetic"><link name="base_link"/>']
    links = []
    with zipfile.ZipFile(path, "w") as archive:
        for suffix, x in (("R", -.23), ("L", .23)):
            for index in range(1, 8):
                link = f"Link{index}_{suffix}"
                xml.append(f'<link name="{link}">')
                if index in (1, 4, 7):
                    links.append(link)
                    xml.append(f'<visual><origin xyz="0.003 -0.002 0.004" rpy="0.35 0.45 0.1"/>'
                               f'<geometry><mesh filename="package://test/meshes/{link}.STL" scale="1 1.1 0.9"/></geometry></visual>')
                    archive.writestr(f"pkg/meshes/{link}.STL", stl_bytes(box_triangles(), suffix == "R"))
                xml.append('</link>')
                parent = "base_link" if index == 1 else f"Link{index - 1}_{suffix}"
                xyz = f"{x} -0.18 1.0" if index == 1 else "0 .06 0"
                xml.append(f'<joint name="Joint{index}_{suffix}" type="revolute"><parent link="{parent}"/>'
                           f'<child link="{link}"/><origin xyz="{xyz}"/><axis xyz="0 0 1"/>'
                           '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>')
        archive.writestr("pkg/urdf/model.urdf", "".join(xml) + "</robot>")
    return links


def raster_depth(triangles, T_base_camera, k, shape):
    """Independent triangle z-buffer: pixel-centre barycentrics, perspective-correct z."""
    camera = apply(inverse(T_base_camera), triangles)
    camera = camera[np.all(camera[:, :, 2] > .01, axis=1)]
    projected = camera @ k.T
    uv = projected[:, :, :2] / projected[:, :, 2:]
    # Vectorised bounding boxes keep the vendor mesh smoke test reasonably short.
    lo = np.maximum(np.ceil(uv.min(axis=1)).astype(int), [0, 0])
    hi = np.minimum(np.floor(uv.max(axis=1)).astype(int), [shape[1] - 1, shape[0] - 1])
    depth = np.full(shape, np.inf)
    for triangle, polygon, minimum, maximum in zip(camera, uv, lo, hi):
        if np.any(maximum < minimum):
            continue
        a, b, c = polygon
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-12:
            continue
        x0, y0 = minimum
        x1, y1 = maximum
        y, x = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        w0 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / den
        w1 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den
        w2 = 1 - w0 - w1
        inside = (w0 >= -1e-10) & (w1 >= -1e-10) & (w2 >= -1e-10)
        reciprocal = w0 / triangle[0, 2] + w1 / triangle[1, 2] + w2 / triangle[2, 2]
        z = np.divide(1, reciprocal, out=np.full(x.shape, np.inf), where=inside & (reciprocal > 0))
        depth[y0:y1 + 1, x0:x1 + 1] = np.minimum(depth[y0:y1 + 1, x0:x1 + 1], z)
    depth[~np.isfinite(depth)] = 0
    return depth


def add_transform(writer, parent, child, matrix):
    msg = tf_message(parent, child, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    rotation = msg.transforms[0].transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = map(float, q)
    writer.add_tf(msg)


def write_snapshot(path, triangles, camera, T_color_depth, shape=(240, 320), unit=.0001):
    k = np.array([[250., 0, shape[1] / 2], [0, 250., shape[0] / 2], [0, 0, 1]])
    depth = raster_depth(triangles, camera @ T_color_depth, k, shape)
    # Deliberately different color dimensions/intrinsics; geometry must use native depth.
    color_shape = (shape[0], shape[1] + 80)
    color_k = k.copy()
    color_k[0, 2] += 40
    color_z = raster_depth(triangles, camera, color_k, color_shape)
    color = np.full(color_shape + (3,), 35, np.uint8)
    color[color_z > 0] = (100, 130, 160)
    raw = np.rint(depth / unit).astype(np.uint16)
    writer = SnapshotWriter(path, 3, unit, "synthetic", "independent z-buffer, static")
    for stream, image, intrinsics in (("color", color, color_k), ("depth", raw, k)):
        for index in range(3):
            msg, info = messages(image, f"d435_{stream}_optical_frame", index + 1, intrinsics)
            writer.save(stream, envelope(msg), envelope(info))
    optical = origin_transform([0, 0, 0], [-np.pi / 2, 0, -np.pi / 2])
    add_transform(writer, "d435_link", "d435_color_frame", np.eye(4))
    add_transform(writer, "d435_color_frame", "d435_color_optical_frame", optical)
    add_transform(writer, "d435_link", "d435_depth_frame", optical @ T_color_depth @ inverse(optical))
    add_transform(writer, "d435_depth_frame", "d435_depth_optical_frame", optical)
    writer.finish()
    return depth


def write_states(path, poses=("p1",), degrees=None):
    degrees = [0.] * 7 if degrees is None else degrees
    path.write_text("".join(f"{pose} sample=1/3 arm={side} frame=123 counter=first mode_raw=0 "
                            f"commanded_state_raw=0 error_raw=0 position_degrees={degrees}\n"
                            for pose in poses for side in ("right", "left")))


class ArmCheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.urdf = cls.root / "model.zip"
        cls.links = synthetic_urdf(cls.urdf)
        cls.states = cls.root / "states.txt"
        write_states(cls.states)
        cls.model = load_urdf(cls.urdf)
        cls.visuals = load_visuals(cls.model, cls.links)
        cls.chains, cls.owners = arm_chains(cls.model, "base_link", ["right=Link7_R", "left=Link7_L"], cls.links)
        cls.meshes = pose_meshes(cls.model, cls.visuals, "base_link", cls.chains, cls.owners, read_states(cls.states), "p1")
        cls.camera = origin_transform([.03, -.01, -.05], [.03, -.05, .02])
        cls.T_color_depth = origin_transform([.025, -.004, .003], [0, .008, .002])
        cls.snapshot = cls.root / "snapshot"
        triangles = np.concatenate([v for links in cls.meshes.values() for v in links.values()])
        cls.expected_depth = write_snapshot(cls.snapshot, triangles, cls.camera, cls.T_color_depth)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.work = tempfile.TemporaryDirectory()
        self.addCleanup(self.work.cleanup)
        self.directory = Path(self.work.name)
        self.camera_file = self.directory / "camera.json"
        self.write_camera(self.camera)
        self.argv = ["arm-check", "--urdf", str(self.urdf), "--base", "base_link",
                     "--arm-chain", "right=Link7_R", "--arm-chain", "left=Link7_L", "--links", ",".join(self.links),
                     "--states", str(self.states), "--pose", f"p1={self.snapshot}", "--camera-pose", str(self.camera_file),
                     "--output", str(self.directory / "result"), "--confirm-static-same-pose",
                     "--surface-sample-spacing-mm", ".6"]
        self.args = parser().parse_args(self.argv)

    def write_camera(self, matrix, **fields):
        self.camera_file.write_text(json.dumps({"units": "m", "from_frame": "d435_color_optical_frame",
                                               "to_frame": "base_link", "matrix": matrix.tolist(), **fields}))

    def test_true_pose_distances_outputs_provenance_and_exclusive(self):
        result = run_arm_check(self.args)
        self.assertEqual(result["status"], "candidate_not_approved")
        for group in result["before"]:
            self.assertGreaterEqual(group["valid_depth_count"], 300)
            self.assertLess(group["unsigned_distance_mm"]["median"], .5)
        self.assertEqual(result["cross_validation"]["usable_groups"], 2)
        self.assertEqual(len(result["cross_validation"]["held_out"]), 2)
        self.assertEqual(result["inputs"]["urdf"]["xml_sha256"], self.model.urdf_sha256)
        self.assertEqual(result["inputs"]["camera_pose"]["sha256"], sha256(self.camera_file))
        self.assertEqual(result["inputs"]["states"]["sha256"], sha256(self.states))
        self.assertEqual(result["inputs"]["snapshots"]["p1"]["snapshot_json_sha256"], sha256(self.snapshot / "snapshot.json"))
        for suffix in ("overlay", "overlay_refined", "depth_residual", "depth_residual_refined"):
            image = cv2.imread(str(Path(self.args.output) / f"pose_p1_{suffix}.png"))
            self.assertIsNotNone(image)
            self.assertGreater(np.count_nonzero(image), 0)
        self.assertEqual(json.loads((Path(self.args.output) / "arm_check.json").read_text()), result)
        with self.assertRaisesRegex(CalibrationError, "existing output"):
            run_arm_check(self.args)

    def test_perturbed_pose_refinement_and_group_cross_validation(self):
        xi = np.r_[np.deg2rad([.8, -1.1, .6]), [.008, -.005, .004]]
        self.write_camera(self.camera @ exp_se3(xi))
        result = run_arm_check(self.args)
        corrected = np.array(result["refinement"]["corrected_T_base_color"])
        error = inverse(self.camera) @ corrected
        self.assertLess(np.linalg.norm(error[:3, 3]) * 1000, 1.)
        self.assertLess(np.rad2deg(np.linalg.norm(Rotation.from_matrix(error[:3, :3]).as_rotvec())), .1)
        self.assertGreater(result["refinement"]["translation_norm_mm"], 5)
        self.assertGreater(result["refinement"]["rotation_norm_deg"], 1)
        self.assertEqual(len(result["refinement"]["observability"]["normalised_eigenvalues"]), 6)
        for group in result["cross_validation"]["held_out"]:
            self.assertLess(group["after_mm"]["median"], group["before_mm"]["median"])
            self.assertLess(group["after_mm"]["p95"], group["before_mm"]["p95"])

    def test_native_depth_tf_median_and_half_valid_rule(self):
        import shutil
        target = self.directory / "snapshot"
        shutil.copytree(self.snapshot, target)
        record = json.loads((target / "snapshot.json").read_text())
        for i, frame in enumerate(record["streams"]["depth"]):
            image = cv2.imread(str(target / frame["file"]), cv2.IMREAD_UNCHANGED)
            image[0, 0] = [0, 1000, 3000][i]
            image[0, 1] = [0, 0, 3000][i]
            cv2.imwrite(str(target / frame["file"]), image)
        snapshot = load_snapshot(target)
        self.assertAlmostEqual(snapshot["depth_m"][0, 0], .2)
        self.assertEqual(snapshot["depth_m"][0, 1], 0)
        np.testing.assert_allclose(snapshot["T_color_depth"], self.T_color_depth, atol=1e-14)
        np.testing.assert_allclose(snapshot["depth_m"][1:], self.expected_depth[1:], atol=.000051)

    def test_binary_ascii_stl_visual_origin_scale_and_plain_urdf(self):
        triangle = box_triangles()
        np.testing.assert_allclose(read_stl(stl_bytes(triangle, True)), triangle, atol=1e-8)
        np.testing.assert_allclose(read_stl(stl_bytes(triangle, False)), triangle, atol=1e-8)
        expected = apply(origin_transform([.003, -.002, .004], [.35, .45, .1]), triangle * [1, 1.1, .9])
        np.testing.assert_allclose(self.visuals["Link1_R"], expected, atol=1e-8)
        with zipfile.ZipFile(self.urdf) as archive:
            archive.extractall(self.directory)
        model = load_urdf(self.directory / "pkg/urdf/model.urdf")
        with self.assertRaisesRegex(CalibrationError, "mesh-root"):
            load_visuals(model, self.links)
        plain = load_visuals(model, self.links, self.directory / "pkg")
        for link in self.links:
            np.testing.assert_allclose(plain[link], self.visuals[link])
        bad = self.directory / "ambiguous.zip"
        with zipfile.ZipFile(self.urdf) as source, zipfile.ZipFile(bad, "w") as archive:
            for name in source.namelist():
                archive.writestr(name, source.read(name))
            archive.writestr("other/meshes/Link1_R.STL", stl_bytes(triangle, False))
        with self.assertRaisesRegex(CalibrationError, "ambiguous mesh"):
            load_visuals(load_urdf(bad), self.links)

    def test_confirmation_frame_unit_link_and_joint_errors(self):
        changes = [("confirm_static_same_pose", False, "confirm-static-same-pose"),
                   ("links", "Missing", "Missing render link"),
                   ("arm_chain", ["right=Link7_R"], "covered by one arm-chain"),
                   ("arm_chain", ["right=Link6_R", "left=Link7_L"], "exactly 7 angular"),
                   ("links", "Link1_R,Link1_R", "unique"),
                   ("icp_gates_mm", "15,60", "decreasing")]
        for field, value, message in changes:
            with self.subTest(field=field, value=value):
                args = copy.copy(self.args)
                setattr(args, field, value)
                with self.assertRaisesRegex(CalibrationError, message):
                    run_arm_check(args)
        for field, value, message in (("units", "mm", "units must be m"), ("to_frame", "wrong", "to_frame"),
                                      ("from_frame", "wrong", "from_frame")):
            with self.subTest(field=field):
                self.write_camera(self.camera, **{field: value})
                with self.assertRaisesRegex(CalibrationError, message):
                    run_arm_check(self.args)
        self.write_camera(self.camera)
        states = self.directory / "states.txt"
        states.write_text(self.states.read_text().splitlines()[0] + "\n")
        self.args.states = str(states)
        with self.assertRaisesRegex(CalibrationError, "Missing joint state"):
            run_arm_check(self.args)
        self.assertFalse(Path(self.args.output).exists())
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main([v for v in self.argv if v != "--confirm-static-same-pose"]), 2)

    def test_states_duplicates_error_and_limits_rejected(self):
        path = self.directory / "states.txt"
        original = self.states.read_text()
        for text, message in ((original * 2, "Duplicate pose/arm"),
                              (original.replace("error_raw=0", "error_raw=1"), "error_raw must be 0"),
                              (original.replace("[0.0,", "[9999.0,"), "outside limits"),
                              (original.replace("[0.0, ", "[", 1), "exactly 7 finite")):
            with self.subTest(message=message):
                path.write_text(text)
                self.args.states = str(path)
                with self.assertRaisesRegex(CalibrationError, message):
                    run_arm_check(self.args)

    def test_snapshot_bad_unit_distortion_header_and_tf_rejected(self):
        import shutil
        target = self.directory / "bad"
        shutil.copytree(self.snapshot, target)
        original = json.loads((target / "snapshot.json").read_text())
        for kind, message in (("unit", "depth_unit_m"), ("distortion", "D=0"), ("header", "headers differ"),
                              ("tf", "Missing static TF"), ("incomplete", "Incomplete snapshot")):
            record = copy.deepcopy(original)
            if kind == "unit":
                record["depth_unit_m"] = 0
            elif kind == "distortion":
                record["streams"]["depth"][0]["camera_info"]["d"][0] = .1
            elif kind == "header":
                record["streams"]["depth"][0]["camera_info"]["header"]["frame_id"] = "wrong"
            elif kind == "tf":
                record["tf_static"] = []
            else:
                record["frames_requested_per_stream"] = 4
            (target / "snapshot.json").write_text(json.dumps(record))
            with self.subTest(kind=kind), self.assertRaisesRegex(CalibrationError, message):
                load_snapshot(target)

    def test_insufficient_groups_and_weak_observability_explicit(self):
        snapshot = load_snapshot(self.snapshot)
        groups = make_groups("p1", snapshot, self.meshes, self.camera, 25, .002, 2.)
        result = cross_validate(groups[:1], self.camera, [.06, .03, .015], 300)
        self.assertEqual(result["status"], "fewer_than_2_usable_groups")
        corrected, fit = refine(groups, self.camera, [.06, .03, .015], 10**8)
        np.testing.assert_array_equal(corrected, self.camera)
        self.assertEqual(fit["status"], "insufficient_points")
        points = np.array([[x, y, 1.] for x in np.linspace(-.2, .2, 10) for y in np.linspace(-.2, .2, 10)])
        obs = observability(points, points, np.tile([0, 0, 1.], (len(points), 1)), np.eye(4))
        self.assertGreaterEqual(obs["weak_directions"], 3)

    def test_residual_colors_largest_gate_and_appended_legend(self):
        samples = np.array([[0., 0., 1.]])
        source = np.array([[0., 0., 1.003], [0., 0., 1.010], [0., 0., 1.025], [0., 0., 1.070]])
        group = Group("p", "right", source, np.array([[0, x] for x in range(4)]), 4,
                      samples, np.array([[0., 0., 1.]]), cKDTree(samples), {"link": 1})
        image = residual_image({"depth_m": np.zeros((2, 4))}, [group], np.eye(4), [.06, .03, .015], "input")
        np.testing.assert_array_equal(image[0, :4], [[0, 200, 0], [0, 255, 255], [0, 0, 255], [128, 128, 128]])
        self.assertEqual(image.shape, (66, 560, 3))
        self.assertGreater(np.count_nonzero(image[2:] != 25), 0)

    def test_even_depth_frame_count_accepts_exactly_half_valid(self):
        target = self.directory / "even"
        writer = SnapshotWriter(target, 4, .001, "test", "half-valid rule")
        for index in range(4):
            for stream in ("color", "depth"):
                image = (np.zeros((2, 2, 3), np.uint8) if stream == "color" else
                         np.array([[[0, 0], [0, 0]], [[0, 0], [0, 0]],
                                   [[1000, 0], [0, 0]], [[3000, 3000], [0, 0]]], np.uint16)[index])
                msg, info = messages(image, stream, index + 1)
                writer.save(stream, envelope(msg), envelope(info))
        writer.add_tf(tf_message())
        writer.finish()
        depth = load_snapshot(target)["depth_m"]
        self.assertEqual(depth[0, 0], 2.)
        self.assertEqual(depth[0, 1], 0.)

    def test_import_has_no_ros_or_qt(self):
        code = ("import sys; import robot_perception.calibration.arm_check; "
                "assert not any(n in sys.modules for n in ('rclpy','sensor_msgs','tf2_msgs','PyQt5','PySide6'))")
        subprocess.run([sys.executable, "-c", code], check=True)


@unittest.skipUnless("ROBOT_PERCEPTION_TIANJI_URDF" in os.environ, "ROBOT_PERCEPTION_TIANJI_URDF is unset")
class RealURDFArmCheckTests(unittest.TestCase):
    def test_vendor_all_arm_meshes_end_to_end(self):
        started = time.monotonic()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # A set but missing/corrupt vendor path is deliberately a test failure, not a skip.
            model = load_urdf(os.environ["ROBOT_PERCEPTION_TIANJI_URDF"])
            links = [f"Link{i}_{suffix}" for suffix in ("R", "L") for i in range(1, 8)]
            visuals = load_visuals(model, links)
            self.assertEqual(len(visuals), 14)
            states_path = root / "states.txt"
            write_states(states_path, degrees=[0., -3., 0., -5., 0., 0., 0.])
            chains, owners = arm_chains(model, "base_link", ["right=Link7_R", "left=Link7_L"], links)
            meshes = pose_meshes(model, visuals, "base_link", chains, owners, read_states(states_path), "p1")
            triangles = np.concatenate([v for group in meshes.values() for v in group.values()])
            # A fixed camera above/in front, looking down at both zero-ish arms.
            center = (triangles.min(axis=(0, 1)) + triangles.max(axis=(0, 1))) / 2
            eye = center + [.65, -.15, .75]
            z = center - eye
            z /= np.linalg.norm(z)
            x = np.cross(z, [0., 0., 1.])
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            camera = np.eye(4)
            camera[:3, :3] = np.column_stack((x, y, z))
            camera[:3, 3] = eye
            snapshot = root / "snapshot"
            write_snapshot(snapshot, triangles, camera, origin_transform([.025, 0, 0], [0, .005, 0]), unit=.001)
            camera_path = root / "camera.json"
            camera_path.write_text(json.dumps({"units": "m", "from_frame": "d435_color_optical_frame",
                                              "to_frame": "base_link", "matrix": camera.tolist()}))
            args = parser().parse_args(["arm-check", "--urdf", model.urdf_path, "--base", "base_link",
                                        "--arm-chain", "right=Link7_R", "--arm-chain", "left=Link7_L",
                                        "--links", ",".join(links), "--states", str(states_path), "--pose", f"p1={snapshot}",
                                        "--camera-pose", str(camera_path), "--output", str(root / "result"),
                                        "--confirm-static-same-pose"])
            result = run_arm_check(args)
            self.assertEqual(result["status"], "candidate_not_approved")
            self.assertEqual(result["cross_validation"]["usable_groups"], 2)
            self.assertEqual(sum(len(g["surface_samples_per_link"]) for g in result["before"]), 14)
            for group in result["before"]:
                self.assertGreaterEqual(group["valid_depth_count"], 300)
                self.assertLess(group["unsigned_distance_mm"]["median"], 2.)
            self.assertTrue((root / "result/pose_p1_overlay_refined.png").is_file())
        print(f"Real-URDF arm-check smoke runtime: {time.monotonic() - started:.3f}s", flush=True)
