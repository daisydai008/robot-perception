"""Installed-package migration checks; no checkout or hardware import fallback."""
import hashlib
from importlib.resources import files
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from robot_perception.calibration.common import CalibrationError, read_json
from robot_perception.calibration.extrinsics import defaults, digest
from robot_perception.calibration.extrinsics_session import Session


# Subprocesses fail on an attempted optional import, not just loaded modules.
IMPORT_GUARD = '''
import importlib.abc
import sys
class RejectOptional(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy', 'sensor_msgs', 'PyQt5', 'PySide6',
                                     'torch', 'tensorflow', 'pyrealsense2'}:
            raise AssertionError('Unexpected optional import: ' + fullname)
sys.meta_path.insert(0, RejectOptional())
'''


class PackagingTests(unittest.TestCase):
    def run_isolated(self, code):
        env = dict(os.environ, PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory(prefix="robot-perception-import-", dir="/tmp") as tmp:
            return subprocess.run([sys.executable, "-c", code], cwd=tmp, env=env,
                                  text=True, capture_output=True, timeout=30)

    def test_core_and_entry_imports_do_not_load_optional_layers(self):
        code = IMPORT_GUARD + '''
import importlib
for name in ('robot_perception', 'robot_perception.ui', 'robot_perception.calibration',
             'robot_perception.calibration.common', 'robot_perception.calibration.calibrate',
             'robot_perception.calibration.capture', 'robot_perception.calibration.offline',
             'robot_perception.calibration.extrinsics',
             'robot_perception.calibration.extrinsics_session',
             'robot_perception.calibration.extrinsics_live',
             'robot_perception.calibration.extrinsics_ui'):
    importlib.import_module(name)
assert 'robot_perception.ui._extrinsics_window' not in sys.modules
'''
        result = self.run_isolated(code)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_python_module_help_without_qt_or_ros(self):
        for module in ("robot_perception", "robot_perception.calibration.calibrate",
                       "robot_perception.calibration.extrinsics_ui"):
            with self.subTest(module=module):
                code = IMPORT_GUARD + f'''
import runpy
sys.argv = [{module!r}, '--help']
try:
    runpy.run_module({module!r}, run_name='__main__')
except SystemExit as exc:
    assert exc.code == 0, exc.code
assert not any(n in sys.modules for n in ('cv2', 'numpy', 'scipy', 'PyQt5', 'rclpy'))
'''
                result = self.run_isolated(code)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage:", result.stdout)

    def test_console_scripts_help_and_example_rejection(self):
        code = IMPORT_GUARD + '''
from importlib.metadata import distribution
from importlib.resources import as_file, files
entries = {e.name: e for e in distribution('robot-perception').entry_points
           if e.group == 'console_scripts'}
assert set(entries) == {'robot-calibrate', 'robot-extrinsics-ui'}
for entry in entries.values():
    try:
        entry.load()(['--help'])
    except SystemExit as exc:
        assert exc.code == 0, exc.code
    else:
        raise AssertionError('help did not exit')
with as_file(files('robot_perception.calibration').joinpath('targets.example.json')) as path:
    assert entries['robot-calibrate'].load()(['check-config', '--config', str(path), '--formal']) == 2
assert 'PyQt5' not in sys.modules and 'rclpy' not in sys.modules
'''
        result = self.run_isolated(code)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status": "blocked"', result.stdout)

    def test_missing_qt_is_an_explicit_ui_error_not_an_import_requirement(self):
        code = IMPORT_GUARD.replace("raise AssertionError('Unexpected optional import: ' + fullname)",
                                    "raise ImportError('optional layer unavailable: ' + fullname)") + '''
from robot_perception.calibration.extrinsics_ui import main
assert 'PyQt5' not in sys.modules
assert main([]) == 2
assert 'rclpy' not in sys.modules
'''
        result = self.run_isolated(code)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("UI requires PyQt5", result.stderr)

    def test_installed_example_resource_matches_migration_source(self):
        resource = files("robot_perception.calibration").joinpath("targets.example.json")
        self.assertEqual(hashlib.sha256(resource.read_bytes()).hexdigest(),
                         "fa0e75e40b65252d5fb8de8e2b9610a29d7e461c2fe52aa90add182696cbdcb2")
        self.assertIsNone(json.loads(resource.read_text())["board"]["square_length_m"])

    def test_legacy_empty_session_v1_and_external_compatibility(self):
        # Original schema v1 has no migration step; opened results remain invalid.
        inputs = {"schema_version": 1, "kind": "fixed_camera_checkerboard_session",
                  "config": defaults(), "samples": [], "split_freeze": None}
        with tempfile.TemporaryDirectory(prefix="robot-perception-legacy-", dir="/tmp") as tmp:
            directory = Path(tmp)
            (directory / "session.json").write_text(json.dumps({**inputs, "input_sha256": digest(inputs)}))
            opened = Session.open(directory)
            self.assertEqual(opened.inputs(), inputs)
            self.assertIsNone(opened.result)
            with self.assertRaises(CalibrationError):
                opened.export(directory / "candidate.json")
        if os.environ.get("ROBOT_PERCEPTION_FIXTURE_DIR"):
            directory = Path(os.environ["ROBOT_PERCEPTION_FIXTURE_DIR"]) / "extrinsics_Q12_setup_20261109_214759"
            path = directory / "session.json"
            before = path.read_bytes()
            self.assertEqual(hashlib.sha256(before).hexdigest(),
                             "9876fa4b0a7f90775b2acfcdb33f7359ec1d772517352d8fe394dc25a1c3ce94")
            opened = Session.open(directory)
            self.assertEqual(opened.config, read_json(path)["config"])
            self.assertEqual(opened.samples, [])
            self.assertIsNone(opened.result)
            from robot_perception.calibration.extrinsics_ui import Window
            from PyQt5.QtWidgets import QApplication
            app = QApplication.instance() or QApplication([])
            window = Window(opened)
            try:
                window.show()
                app.processEvents()
                self.assertIsNone(window.live)
                self.assertFalse(window.export_button.isEnabled())
                with tempfile.TemporaryDirectory(prefix="robot-perception-revision-", dir="/tmp") as tmp:
                    revision = Path(tmp) / "revision"
                    opened.save(revision)
                    self.assertEqual(Session.open(revision).inputs(), opened.inputs())
            finally:
                window.close()
                app.processEvents()
            self.assertFalse(window.detector.is_alive())
            self.assertEqual(path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
