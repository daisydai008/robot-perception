"""Offline URDF FK/state-reference tests; vendor material is read only via opt-in path."""
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation

from robot_perception.calibration.calibrate import main
from robot_perception.calibration.common import CalibrationError
from robot_perception.calibration.extrinsics import defaults, se3, validate_reference
from robot_perception.calibration.urdf_fk import load_urdf, parse_samples, parse_state_line, parse_urdf


def joint(name, parent, child, kind="fixed", extra=""):
    return (f'<joint name="{name}" type="{kind}"><parent link="{parent}"/>'
            f'<child link="{child}"/>{extra}</joint>')


def urdf(links, joints):
    return ('<robot name="synthetic">' + ''.join(f'<link name="{name}"/>' for name in links)
            + ''.join(joints) + '</robot>').encode()


LIMIT = '<limit lower="-3.2" upper="3.2"/>'
CONTROL_STATE = ('arm=right frame=42 mode_raw=0 commanded_state_raw=1 error_raw=0 '
                 'position_degrees=[1.0, -2.0, 3.0, -4.0, 5.0, 6.0, -7.0]')
CLI_STATE = 'sample=1/3 ' + CONTROL_STATE.replace('frame=42 ', 'frame=42 counter=first ')


def seven_joint_urdf(kind="revolute", count=7):
    # Include a fixed joint to ensure only movable joints consume values, in chain order.
    return urdf(['base', 'mount'] + [f'link{i}' for i in range(1, count + 1)],
                [joint('mount', 'base', 'mount', extra='<origin xyz="0 0 1"/>')]
                + [joint(f'j{i}', 'mount' if i == 1 else f'link{i-1}', f'link{i}', kind,
                         '<origin xyz="0.1 0 0"/><axis xyz="0 0 2"/>' + LIMIT)
                   for i in range(1, count + 1)])


class URDFFKTests(unittest.TestCase):
    def test_rpy_fixed_axis_convention(self):
        rpy = [.37, -.68, 1.2]
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', extra=
                          '<origin xyz="0.3 -0.4 0.8" rpy=".37 -.68 1.2"/>')]))
        actual = model.fk('a', 'b', {})
        np.testing.assert_allclose(actual[:3, :3], Rotation.from_euler('xyz', rpy).as_matrix(), atol=1e-15)
        np.testing.assert_allclose(actual[:3, 3], [.3, -.4, .8])

    def test_revolute_normalised_arbitrary_axis_after_origin(self):
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', 'revolute',
                          '<origin xyz=".1 .2 .3" rpy=".4 -.2 .7"/><axis xyz="2 -3 4"/>' + LIMIT)]))
        axis = np.array([2., -3., 4.]) / np.linalg.norm([2, -3, 4])
        np.testing.assert_allclose(model.joints['j'].axis, axis)
        actual = model.fk('a', 'b', {'j': .8})
        expected = Rotation.from_euler('xyz', [.4, -.2, .7]).as_matrix() @ Rotation.from_rotvec(axis * .8).as_matrix()
        np.testing.assert_allclose(actual[:3, :3], expected, atol=1e-15)
        np.testing.assert_allclose(actual[:3, 3], [.1, .2, .3])

    def test_continuous_default_axis_and_origin(self):
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', 'continuous')]))
        value = 12 * math.pi + .3
        actual = model.fk('a', 'b', {'j': value})
        np.testing.assert_allclose(actual[:3, :3], Rotation.from_rotvec([value, 0, 0]).as_matrix(), atol=1e-15)
        np.testing.assert_array_equal(actual[:3, 3], [0, 0, 0])

    def test_prismatic_metres_in_rotated_joint_frame(self):
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', 'prismatic',
                          '<origin xyz=".3 .4 .5" rpy="0 0 1.5707963267948966"/>'
                          '<axis xyz="2 0 0"/><limit lower="-.1" upper=".2"/>')]))
        actual = model.fk('a', 'b', {'j': .12})
        np.testing.assert_allclose(actual[:3, 3], [.3, .52, .5], atol=1e-15)
        with self.assertRaisesRegex(CalibrationError, 'outside limits'):
            model.fk('a', 'b', {'j': .20001})

    def test_fixed_chain_zero_axis_and_identity_chain(self):
        model = parse_urdf(urdf(['a', 'b', 'c'], [joint('j1', 'a', 'b', extra='<origin xyz=".1 .2 .3"/><axis xyz="2 0 0"/>'),
                          joint('j2', 'b', 'c', extra='<origin xyz=".4 0 0"/><axis xyz="0 0 0"/>')]))
        expected = np.eye(4)
        expected[:3, 3] = [.5, .2, .3]
        np.testing.assert_allclose(model.fk('a', 'c', {}), expected)
        np.testing.assert_array_equal(model.fk('b', 'b', {}), np.eye(4))
        self.assertEqual(model.chain('a', 'c')[0].name, 'j1')
        np.testing.assert_array_equal(model.joints['j1'].axis, [1, 0, 0])

    def test_analytic_two_link_planar(self):
        model = parse_urdf(urdf(['base', 'l1', 'l2', 'tip'], [
            joint('j1', 'base', 'l1', 'revolute', '<axis xyz="0 0 1"/>' + LIMIT),
            joint('j2', 'l1', 'l2', 'revolute', '<origin xyz=".7 0 0"/><axis xyz="0 0 1"/>' + LIMIT),
            joint('tip', 'l2', 'tip', extra='<origin xyz=".4 0 0"/>')]))
        for q1, q2 in [(0, 0), (.4, -.7), (math.pi / 2, .8)]:
            actual = model.fk('base', 'tip', {'j1': q1, 'j2': q2})
            np.testing.assert_allclose(actual[:3, 3], [.7 * math.cos(q1) + .4 * math.cos(q1 + q2),
                                                       .7 * math.sin(q1) + .4 * math.sin(q1 + q2), 0], atol=1e-15)
            np.testing.assert_allclose(actual[:3, :3], Rotation.from_euler('z', q1 + q2).as_matrix(), atol=1e-15)

    def test_limits_tiny_tolerance_only(self):
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', 'revolute', LIMIT)]))
        for value in (-3.2, 3.2, 3.2 + 5e-13):
            model.fk('a', 'b', {'j': value})
        for value in (-3.2 - 1e-9, 3.2 + 1e-9):
            with self.assertRaisesRegex(CalibrationError, 'outside limits'):
                model.fk('a', 'b', {'j': value})

    def test_missing_links_and_target_not_under_base(self):
        with self.assertRaisesRegex(CalibrationError, 'missing referenced link'):
            parse_urdf(urdf(['a'], [joint('j', 'a', 'b')]))
        model = parse_urdf(urdf(['a', 'b', 'c'], [joint('j', 'a', 'b')]))
        for base, target in [('missing', 'b'), ('a', 'missing'), ('a', 'c'), ('b', 'a')]:
            with self.subTest(base=base, target=target), self.assertRaises(CalibrationError):
                model.chain(base, target)

    def test_branches_allowed_but_multiple_parents_rejected(self):
        model = parse_urdf(urdf(['a', 'b', 'c'], [joint('j1', 'a', 'b'), joint('j2', 'a', 'c')]))
        self.assertEqual([j.name for j in model.chain('a', 'b')], ['j1'])
        with self.assertRaisesRegex(CalibrationError, 'not under base'):
            model.chain('b', 'c')
        with self.assertRaisesRegex(CalibrationError, 'multiple parent'):
            parse_urdf(urdf(['a', 'b', 'c'], [joint('j1', 'a', 'c'), joint('j2', 'b', 'c')]))

    def test_cycles_rejected_even_in_unrelated_branch(self):
        for joints in ([joint('self', 'a', 'a')],
                       [joint('j1', 'a', 'b'), joint('j2', 'b', 'a')]):
            with self.subTest(joints=joints), self.assertRaisesRegex(CalibrationError, 'cycle'):
                parse_urdf(urdf(['base', 'target', 'a', 'b'], [joint('ok', 'base', 'target')] + joints))

    def test_nonfinite_and_wrong_joint_inputs_rejected(self):
        model = parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', 'continuous')]))
        for positions in ({}, {'wrong': 0}, {'j': 0, 'extra': 1}, {'j': float('nan')},
                          {'j': float('inf')}, {'j': -float('inf')}, {'j': '0'}, {'j': True}):
            with self.subTest(positions=positions), self.assertRaises(CalibrationError):
                model.fk('a', 'b', positions)

    def test_invalid_urdf_fields_rejected(self):
        for kind, extra in [('revolute', ''), ('floating', ''),
                            ('revolute', LIMIT + '<axis xyz="0 0 0"/>'),
                            ('continuous', '<axis xyz="nan 0 1"/>'),
                            ('fixed', '<origin xyz="0 inf 0"/>'), ('fixed', '<origin rpy="0 0 -inf"/>'),
                            ('fixed', '<origin xyz="0 0"/>'), ('fixed', '<origin rpy="wrong"/>'),
                            ('prismatic', '<limit lower="2" upper="1"/>'),
                            ('revolute', '<limit lower="-inf" upper="1"/>')]:
            with self.subTest(kind=kind, extra=extra), self.assertRaises(CalibrationError):
                parse_urdf(urdf(['a', 'b'], [joint('j', 'a', 'b', kind, extra)]))
        for xml in (b'<broken', b'<not_robot/>', b'<?xml version="1.0" encoding="unknown"?><robot/>',
                    urdf(['a', 'a'], []),
                    urdf(['a', 'b', 'c'], [joint('same', 'a', 'b'), joint('same', 'a', 'c')])):
            with self.subTest(xml=xml), self.assertRaises(CalibrationError):
                parse_urdf(xml)

    def test_file_and_zip_loading_hash_and_member_selection(self):
        xml = seven_joint_urdf()
        with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
            path = Path(tmp) / 'robot.urdf'
            path.write_bytes(xml)
            model = load_urdf(path)
            self.assertEqual(model.urdf_sha256, hashlib.sha256(xml).hexdigest())
            self.assertEqual(model.urdf_path, str(path))
            self.assertIsNone(model.urdf_member)
            with self.assertRaises(CalibrationError):
                load_urdf(path, 'robot.urdf')
            archive = Path(tmp) / 'robot.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('vendor/robot.urdf', xml)
                z.writestr('mesh.stl', 'not read')
            model = load_urdf(archive)
            self.assertEqual(model.urdf_member, 'vendor/robot.urdf')
            self.assertEqual(model.urdf_sha256, hashlib.sha256(xml).hexdigest())
            with self.assertRaises(CalibrationError):
                load_urdf(archive, 'missing.urdf')
            with zipfile.ZipFile(archive, 'a') as z:
                z.writestr('other.urdf', xml)
            with self.assertRaisesRegex(CalibrationError, 'exactly one'):
                load_urdf(archive)
            self.assertEqual(load_urdf(archive, 'other.urdf').urdf_member, 'other.urdf')
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('not-urdf.txt', 'no URDF')
            with self.assertRaises(CalibrationError):
                load_urdf(archive)
            archive.write_bytes(b'broken zip')
            with self.assertRaises(CalibrationError):
                load_urdf(archive)

    def test_import_numpy_stdlib_only(self):
        code = '''
import importlib.abc
import sys
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'scipy', 'cv2', 'PyQt5', 'rclpy', 'sensor_msgs', 'tianji_control'}:
            raise AssertionError('Unexpected import: ' + fullname)
sys.meta_path.insert(0, Guard())
import robot_perception.calibration.urdf_fk
'''
        env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
        env.pop('PYTHONPATH', None)
        result = subprocess.run([sys.executable, '-c', code], cwd='/tmp', env=env,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)


class StateLineTests(unittest.TestCase):
    def test_cli_and_control_formats_and_optional_fields(self):
        for line in (CLI_STATE, CONTROL_STATE):
            parsed = parse_state_line(line, 'right')
            self.assertEqual(parsed['frame'], '42')
            self.assertEqual(parsed['position_degrees'], [1, -2, 3, -4, 5, 6, -7])
            self.assertEqual(parsed['state_line'], line)
        parsed = parse_state_line('arm=left position_degrees=[0, +1, -.2, 3e-2, 4., 5E+1, 6]', 'left')
        self.assertEqual(parsed['frame'], 'n/a')
        self.assertEqual(parsed['position_degrees'], [0, 1, -.2, .03, 4, 50, 6])

    def test_arm_count_error_and_malformed_states(self):
        invalid = [CONTROL_STATE.replace('right', 'left'), CONTROL_STATE.replace('error_raw=0', 'error_raw=1'),
                   CONTROL_STATE.replace('error_raw=0', 'error_raw=nan'),
                   CONTROL_STATE.replace('frame=42', 'frame=bad'), CONTROL_STATE.replace('arm=right ', ''),
                   CONTROL_STATE.replace('1.0, ', ''), CONTROL_STATE.replace('1.0, ', '1.0, 1.0, '),
                   CONTROL_STATE.replace('1.0', 'nan'), CONTROL_STATE.replace('1.0', 'inf'),
                   CONTROL_STATE.replace('1.0', '1e999'), CONTROL_STATE.replace('1.0', 'True'),
                   CONTROL_STATE.replace('1.0', '1_0'), CONTROL_STATE[:-1], CONTROL_STATE + ' junk',
                   CONTROL_STATE + ' error_raw=0', CONTROL_STATE + 'error_raw=0',
                   CONTROL_STATE.replace(', -2.0', ' -2.0'), 'not a state', '']
        for line in invalid:
            with self.subTest(line=line), self.assertRaises(CalibrationError):
                parse_state_line(line, 'right')

    def test_samples_comments_and_unique_ids(self):
        samples = parse_samples(f'\n# header\n  # comment\n000000 {CONTROL_STATE}\npose-2 {CLI_STATE}\n', 'right')
        self.assertEqual([s['sample_id'] for s in samples], ['000000', 'pose-2'])
        for text in (f'a {CONTROL_STATE}\na {CLI_STATE}', 'a', '# empty\n', ''):
            with self.subTest(text=text), self.assertRaises(CalibrationError):
                parse_samples(text, 'right')

    def test_unsafe_sample_ids(self):
        for sample_id in ('../escape', '/absolute', '.hidden', '..', r'a\b', 'a:b', 'a*', 'a' * 201):
            with self.subTest(sample_id=sample_id), self.assertRaises(CalibrationError):
                parse_samples(f'{sample_id} {CONTROL_STATE}', 'right')


class EEReferenceCLITests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='robot-perception-fk-', dir='/tmp')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.urdf = self.root / 'robot.urdf'
        self.urdf.write_bytes(seven_joint_urdf())
        self.samples = self.root / 'samples.txt'
        self.samples.write_text(f'000000 {CONTROL_STATE}\n000001 {CLI_STATE}\n')
        self.out = self.root / 'out'
        self.args = ['ee-reference', '--urdf', str(self.urdf), '--base', 'base', '--ee', 'link7',
                     '--arm', 'right', '--samples', str(self.samples), '--out-dir', str(self.out)]

    def run_cli(self, confirm=True):
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = main(self.args + (['--confirm-settled-same-pose'] if confirm else []))
        return code, stdout.getvalue(), stderr.getvalue()

    def test_refuses_without_confirmation_before_reading_inputs(self):
        self.urdf.unlink()
        code, _, stderr = self.run_cli(confirm=False)
        self.assertEqual(code, 2)
        self.assertIn('--confirm-settled-same-pose', stderr)
        self.assertFalse(self.out.exists())

    def test_generated_json_matches_fk_and_reference_contract(self):
        code, stdout, stderr = self.run_cli()
        self.assertEqual(code, 0, stderr)
        self.assertIn('2 candidate references', stdout)
        cfg = defaults()
        cfg['mode'] = 'eye_to_hand'
        cfg['frames'].update(ee='link7', base='base')
        model = load_urdf(self.urdf)
        names = [f'j{i}' for i in range(1, 8)]
        records = []
        for index, line in enumerate((CONTROL_STATE, CLI_STATE)):
            ref = json.loads((self.out / f'{index:06d}.json').read_text())
            matrix = validate_reference(ref, cfg)
            degrees = [1, -2, 3, -4, 5, 6, -7]
            np.testing.assert_allclose(matrix, model.fk('base', 'link7', dict(zip(names, np.deg2rad(degrees)))))
            self.assertEqual(ref['provenance'], {
                'tool': 'robot_perception.calibration.urdf_fk', 'urdf_path': str(self.urdf), 'urdf_member': None,
                'urdf_sha256': hashlib.sha256(self.urdf.read_bytes()).hexdigest(), 'urdf_robot_name': 'synthetic',
                'base': 'base', 'ee': 'link7', 'arm': 'right', 'joint_names': names,
                'position_degrees': degrees, 'state_line': line,
                'status': 'fk_from_vendor_urdf_not_validated_against_robot'})
            self.assertTrue(ref['settled_same_pose'])
            self.assertTrue(ref['source_record'].startswith(f'{index:06d}: tianji state frame=42 position_degrees='))
            self.assertIn(f'sha256={model.urdf_sha256}', ref['source_record'])
            records.append(ref['source_record'])
        self.assertEqual(len(set(records)), 2)

    def test_exclusive_create_preflights_entire_batch(self):
        self.out.mkdir()
        existing = self.out / '000001.json'
        existing.write_text('do not overwrite')
        code, _, stderr = self.run_cli()
        self.assertEqual(code, 2)
        self.assertIn('Refusing existing output', stderr)
        self.assertEqual(existing.read_text(), 'do not overwrite')
        self.assertFalse((self.out / '000000.json').exists())
        existing.unlink()
        existing.symlink_to(self.root / 'missing-target')
        self.assertEqual(self.run_cli()[0], 2)
        self.assertFalse((self.root / 'missing-target').exists())
        self.assertFalse((self.out / '000000.json').exists())

    def test_exclusive_create_survives_preflight_race(self):
        original_open = Path.open
        target = self.out / '000000.json'

        def raced_open(path, mode='r', *args, **kwargs):
            if path == target and mode == 'x':
                with original_open(path, 'w') as stream:
                    stream.write('concurrent writer')
            return original_open(path, mode, *args, **kwargs)

        with patch.object(Path, 'open', raced_open):
            self.assertEqual(self.run_cli()[0], 2)
        self.assertEqual(target.read_text(), 'concurrent writer')
        self.assertFalse((self.out / '000001.json').exists())

    def test_invalid_later_line_or_limit_writes_nothing(self):
        for bad in ('malformed', CONTROL_STATE.replace('1.0', '1000.0'),
                    CONTROL_STATE.replace('error_raw=0', 'error_raw=5')):
            self.samples.write_text(f'000000 {CONTROL_STATE}\n000001 {bad}\n')
            self.assertEqual(self.run_cli()[0], 2)
            self.assertFalse(self.out.exists())

    def test_rejects_wrong_chain_count_and_prismatic_degrees(self):
        self.args[self.args.index('link7')] = 'link6'
        self.assertEqual(self.run_cli()[0], 2)
        self.assertFalse(self.out.exists())
        self.args[self.args.index('link6')] = 'link7'
        self.urdf.write_bytes(seven_joint_urdf('prismatic'))
        code, _, stderr = self.run_cli()
        self.assertEqual(code, 2)
        self.assertIn('prismatic metres', stderr)
        self.assertFalse(self.out.exists())

    def test_continuous_chain_and_frame_na(self):
        self.urdf.write_bytes(seven_joint_urdf('continuous'))
        self.samples.write_text('pose arm=right position_degrees=[3600, 0, 0, 0, 0, 0, 0]\n')
        code, _, stderr = self.run_cli()
        self.assertEqual(code, 0, stderr)
        ref = json.loads((self.out / 'pose.json').read_text())
        self.assertIn('frame=n/a', ref['source_record'])
        se3(ref['matrix'])

    def test_zip_member_provenance_and_corrupt_input_exit_two(self):
        archive = self.root / 'robot.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            z.writestr('model.urdf', self.urdf.read_bytes())
            z.writestr('other.urdf', b'<invalid/>')
        self.args[self.args.index(str(self.urdf))] = str(archive)
        self.assertEqual(self.run_cli()[0], 2)
        self.assertFalse(self.out.exists())
        self.args += ['--urdf-member', 'model.urdf']
        code, _, stderr = self.run_cli()
        self.assertEqual(code, 0, stderr)
        ref = json.loads((self.out / '000000.json').read_text())
        self.assertEqual(ref['provenance']['urdf_member'], 'model.urdf')
        self.assertIn('URDF model.urdf sha256=', ref['source_record'])
        archive.write_bytes(b'broken')
        self.assertEqual(self.run_cli()[0], 2)


class RealTianjiURDFTests(unittest.TestCase):
    def test_vendor_right_chain_zero_origins_and_se3(self):
        path = os.environ.get('ROBOT_PERCEPTION_TIANJI_URDF')
        if path is None:
            self.skipTest('Set ROBOT_PERCEPTION_TIANJI_URDF to the vendor zip (read only)')
        # An explicitly configured missing/corrupt fixture must fail, never skip.
        model = load_urdf(path)
        self.assertEqual(model.robot_name, 'Pro-m6-696')
        chain = model.chain('base_link', 'Link7_R')
        self.assertEqual([j.name for j in chain], [f'Joint{i}_R' for i in range(1, 8)])
        self.assertTrue(all(j.kind == 'revolute' for j in chain))
        head = model.joints['Head depth_Joint']
        np.testing.assert_allclose(head.xyz, [0, -.103394958759943, 1.24613640461494], atol=1e-15, rtol=0)
        np.testing.assert_allclose(head.rpy, [1.13446401379639, 0, 0], atol=1e-15, rtol=0)
        expected = np.eye(4)
        for j in chain:
            origin = np.eye(4)
            origin[:3, :3] = Rotation.from_euler('xyz', j.rpy).as_matrix()
            origin[:3, 3] = j.xyz
            expected = expected @ origin
        actual = model.fk('base_link', 'Link7_R', {j.name: 0.0 for j in chain})
        np.testing.assert_allclose(actual, expected, atol=2e-15, rtol=0)
        rng = np.random.default_rng(81)
        for _ in range(50):
            actual = model.fk('base_link', 'Link7_R', {j.name: rng.uniform(j.lower, j.upper) for j in chain})
            se3(actual)
            np.testing.assert_allclose(actual[:3, :3].T @ actual[:3, :3], np.eye(3), atol=2e-15)
            self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1., places=14)
            np.testing.assert_array_equal(actual[3], [0, 0, 0, 1])


if __name__ == '__main__':
    unittest.main()
