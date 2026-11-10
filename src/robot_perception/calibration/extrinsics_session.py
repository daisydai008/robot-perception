"""Auditable, exclusive-write fixed-camera sessions; no ROS or Qt dependencies."""
import copy
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np

from .common import read_json, require, write_json
from .extrinsics import (camera_info, defaults, digest, planar_candidates, profile, solve, solve_inputs,
                        validate_config, validate_header, validate_reference)


def png_bytes(image):
    ok, png = cv2.imencode(".png", image)
    require(ok, "PNG encoding failed")
    return png.tobytes()


def png_image(data):
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None, "Invalid PNG")
    return image


def capture_signature(config):
    b = config["board"]
    return digest({"mode": config["mode"], "frames": config["frames"],
                   "raw_model_confirmed": config["raw_model_confirmed"],
                   "board": {k: b[k] for k in ("squares_x", "squares_y", "square_m", "origin",
                                               "scale_confirmed", "measurement_record")}})


def validate_packet(packet, config, live_age_s=None):
    info = packet["camera_info"]
    camera_info(info)
    validate_header(packet["image_header"])
    require(packet["image_header"] == info["header"], "Exact image/CameraInfo timestamp + frame matching required")
    require(info["header"]["frame_id"] == config["frames"]["color"], "Color optical frame differs from setup")
    require(packet["image_geometry"] == "raw_color_no_crop_no_resize", "Raw image geometry declaration required")
    image = packet["image"]
    require(image.dtype == np.uint8 and image.shape == (info["height"], info["width"], 3), "Image size/type mismatch")
    require(isinstance(packet["provenance"], dict) and bool(packet["provenance"]), "Image provenance required")
    if live_age_s is not None:
        require(np.isfinite(live_age_s) and live_age_s > 0, "Positive freshness bound required")
        require(packet.get("source") == "live", "Not a live packet")
        for key in ("image_receipt_monotonic_ns", "info_receipt_monotonic_ns"):
            age = (time.monotonic_ns() - packet[key]) / 1e9
            require(0 <= age <= live_age_s, "Stale live capture / host receipt")
        stamp = validate_header(packet["image_header"])
        require(abs(time.time_ns() - stamp) / 1e9 <= live_age_s,
                "Stale source timestamp or unsynchronised source/host wall clocks")
    return profile(info)


def load_offline(image_path, metadata_path, full_frame_confirmed=False):
    image_path, metadata_path = Path(image_path), Path(metadata_path)
    raw = read_json(metadata_path)
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    require(image is not None, f"Cannot load image: {image_path}")
    provenance = {"image_path": str(image_path.resolve()),
                  "source_image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                  "metadata_path": str(metadata_path.resolve()),
                  "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest()}
    if "snapshots" in raw:
        records = [r for r in raw["snapshots"] if r["stream"] == "color" and r["file"] == image_path.name]
        require(len(records) == 1, "Select the color PNG named by snapshots.json")
        require(image_path.resolve().parent == metadata_path.resolve().parent, "Snapshot PNG must be beside manifest")
        require(full_frame_confirmed is True,
                "Diagnostic manifest omits ROI/binning/raw declaration: explicitly confirm full raw frame")
        record = records[0]
        info = copy.deepcopy(record["camera_info"])
        info["header"] = record["header"]
        info.setdefault("binning_x", 0)
        info.setdefault("binning_y", 0)
        info.setdefault("roi", {"x_offset": 0, "y_offset": 0, "width": 0, "height": 0, "do_rectify": False})
        raw = {"camera_info": info, "image_header": record["header"],
               "image_geometry": "raw_color_no_crop_no_resize", "diagnostic_record": record}
        provenance["omitted_roi_binning_raw_confirmed_by_operator"] = True
    # Standard sidecar retains arbitrary acquisition metadata, including host receipt stamps.
    return {**raw, "image": image, "source": "offline", "provenance": provenance,
            "import_receipt_unix_ns": time.time_ns()}


class Session:
    def __init__(self, config=None):
        self.config = copy.deepcopy(config if config is not None else defaults())
        validate_config(self.config)
        self.samples = []
        self.images = {}
        self.result = None
        self.split_freeze = None

    def invalidate(self):
        self.result = None

    def configure(self, config):
        validate_config(config)
        require(not self.samples or capture_signature(config) == capture_signature(self.config),
                "Existing captures bind mode, frames and geometry: start a new session to change these")
        if config != self.config:
            self.config = copy.deepcopy(config)
            self.invalidate()

    def add(self, packet, detection, live_age_s=None):
        require(self.split_freeze is None, "Split already frozen: new session/protocol required to add samples")
        validate_packet(packet, self.config, live_age_s)
        if self.samples:
            require(profile(packet["camera_info"]) == profile(self.samples[0]["camera_info"]),
                    "CameraInfo/profile changed: new session required")
        data = png_bytes(packet["image"])
        sha = hashlib.sha256(data).hexdigest()
        require(all(s["image_sha256"] != sha for s in self.samples), "Duplicate image content")
        sample_id = f"{len(self.samples):06d}"
        metadata = {k: copy.deepcopy(v) for k, v in packet.items() if k != "image"}
        sample = {**copy.deepcopy(detection), "id": sample_id, "split": "train", "excluded": False,
                  "exclusion_reason": "", "selected": None, "candidate_confirmed": False,
                  "orientation_confirmed": False, "reference": None, "metadata": metadata,
                  "camera_info": copy.deepcopy(packet["camera_info"]), "image_sha256": sha,
                  "capture_signature": capture_signature(self.config)}
        self._validate_sample(sample, data)
        self.samples.append(sample)
        self.images[sample_id] = data
        self.invalidate()
        return sample

    def update(self, index, **fields):
        allowed = {"split", "excluded", "exclusion_reason", "selected", "candidate_confirmed",
                   "orientation_confirmed", "reference"}
        require(set(fields) <= allowed, "Unsupported sample edit")
        candidate = {**copy.deepcopy(self.samples[index]), **copy.deepcopy(fields)}
        require(self.split_freeze is None or candidate["split"] == self.samples[index]["split"],
                "Split frozen before first solve; retuning requires a new session/protocol, not independent heldout reuse")
        self._validate_sample(candidate, self.images[candidate["id"]])
        self.samples[index] = candidate
        self.invalidate()

    def reverse_order(self, index):
        s = copy.deepcopy(self.samples[index])
        require(bool(s["corners"]), "No corners to reverse")
        s["corners"] = s["corners"][::-1]
        s["reverse"] = not s["reverse"]
        s["candidates"] = (planar_candidates(s["corners"], s["camera_info"], self.config["board"])
                           if s["candidates"] else [])
        s.update(selected=None, candidate_confirmed=False, orientation_confirmed=False, reference=None)
        self._validate_sample(s, self.images[s["id"]])
        self.samples[index] = s
        self.invalidate()

    def _validate_sample(self, s, data):
        require(isinstance(s["id"], str) and len(s["id"]) == 6 and s["id"].isdigit(), "Invalid sample ID")
        require(s["split"] in ("train", "heldout"), "Invalid split")
        require(all(type(s[k]) is bool for k in ("reverse", "excluded", "orientation_confirmed", "candidate_confirmed")),
                "Invalid sample confirmation type")
        require(isinstance(s["exclusion_reason"], str) and (not s["excluded"] or s["exclusion_reason"].strip()),
                "Exclusion requires a reason")
        require(s["capture_signature"] == capture_signature(self.config), "Capture geometry/frames changed")
        require(hashlib.sha256(data).hexdigest() == s["image_sha256"], "PNG hash mismatch")
        require(s["camera_info"] == s["metadata"]["camera_info"], "CameraInfo metadata mismatch")
        validate_packet({**s["metadata"], "image": png_image(data)}, self.config)
        if s["corners"]:
            corners = np.asarray(s["corners"])
            require(np.all(corners[:, 0] >= 0) and np.all(corners[:, 0] < s["camera_info"]["width"]) and
                    np.all(corners[:, 1] >= 0) and np.all(corners[:, 1] < s["camera_info"]["height"]), "Corners out of image")
            board = self.config["board"]
            from .extrinsics import finite_array, object_points
            finite_array(s["corners"], (len(object_points(board)), 2), "corners")
            candidates = (planar_candidates(s["corners"], s["camera_info"], board)
                          if board["scale_confirmed"] and board["measurement_record"].strip()
                          and self.config["raw_model_confirmed"] else [])
            require(len(candidates) == len(s["candidates"]), "Corrupt IPPE candidate count")
            for a, b in zip(candidates, s["candidates"]):
                require(a["positive_depth"] == b["positive_depth"] and
                        np.allclose(a["T_C_Q"], b["T_C_Q"], atol=1e-7, rtol=1e-7), "Corrupt IPPE pose")
                for key in ("rms_px", "max_px"):
                    require((a[key] is None and b[key] is None) or
                            (a[key] is not None and b[key] is not None and abs(a[key] - b[key]) < 1e-6),
                            "Corrupt reprojection metric")
        else:
            require(s["candidates"] == [], "Candidates without corners")
        if s["selected"] is not None:
            require(type(s["selected"]) is int and 0 <= s["selected"] < len(s["candidates"]), "Invalid selected candidate")
            require(s["candidates"][s["selected"]]["positive_depth"], "Candidate behind camera")
        require(not s["candidate_confirmed"] or s["selected"] is not None, "Confirm only a selected candidate")
        if s["reference"] is not None:
            require(s["orientation_confirmed"] and s["candidate_confirmed"],
                    "Confirm physical origin/direction and IPPE candidate BEFORE pose association")
            validate_reference(s["reference"], self.config)

    def validate(self):
        validate_config(self.config)
        if self.split_freeze is not None:
            require(self.split_freeze["assignments"] == {s["id"]: s["split"] for s in self.samples},
                    "Frozen split assignments changed; new session/protocol required")
            require(isinstance(self.split_freeze["declared_unix_ns"], int) and
                    self.split_freeze["declared_unix_ns"] > 0 and bool(self.split_freeze["protocol_record"].strip()),
                    "Invalid split freeze provenance")
        require(len({s["id"] for s in self.samples}) == len(self.samples), "Duplicate IDs")
        require(len({s["image_sha256"] for s in self.samples}) == len(self.samples), "Duplicate images")
        for s in self.samples:
            self._validate_sample(s, self.images[s["id"]])
            require(profile(s["camera_info"]) == profile(self.samples[0]["camera_info"]), "Mixed camera profiles")

    def inputs(self):
        return {"schema_version": 1, "kind": "fixed_camera_checkerboard_session", "config": self.config,
                "samples": self.samples, "split_freeze": self.split_freeze}

    def freeze_splits(self):
        self.validate()
        solve_inputs(self.config, self.samples)
        if self.split_freeze is None:
            self.split_freeze = {"declared_unix_ns": time.time_ns(),
                                 "protocol_record": self.config["thresholds"]["record"],
                                 "assignments": {s["id"]: s["split"] for s in self.samples},
                                 "note": "Frozen before first solve attempt. Repeated evaluation is not fresh independent acceptance."}
            self.invalidate()

    def run_solve(self):
        self.invalidate()
        self.freeze_splits()
        self.validate()
        result = solve(self.config, self.samples)
        result["input_sha256"] = digest(self.inputs())
        self.result = result
        return result

    def save(self, directory):
        self.validate()
        directory = Path(directory)
        require(not directory.exists(), "Refusing existing session directory; save a new revision")
        directory.mkdir(parents=True, exist_ok=False)
        for s in self.samples:
            with (directory / (s["id"] + ".png")).open("xb") as f:
                f.write(self.images[s["id"]])
        payload = copy.deepcopy(self.inputs())
        payload["input_sha256"] = digest(self.inputs())
        # Results are intentionally separate immutable files, never trusted on open.
        write_json(directory / "session.json", payload)

    @classmethod
    def open(cls, directory):
        directory = Path(directory)
        raw = read_json(directory / "session.json")
        require(raw["schema_version"] == 1 and raw["kind"] == "fixed_camera_checkerboard_session", "Unknown session schema")
        session = cls(raw["config"])
        session.samples = raw["samples"]
        session.split_freeze = raw["split_freeze"]
        require(isinstance(session.samples, list), "Invalid samples")
        for s in session.samples:
            require(isinstance(s["id"], str) and len(s["id"]) == 6 and s["id"].isdigit(), "Unsafe sample filename")
            session.images[s["id"]] = (directory / (s["id"] + ".png")).read_bytes()
        require(digest(session.inputs()) == raw["input_sha256"], "Session input hash mismatch")
        session.validate()
        return session

    def export(self, path):
        require(self.result is not None, "No current result: solve again after edits/open")
        self.validate()
        require(self.result["input_sha256"] == digest(self.inputs()), "Stale result: inputs changed")
        write_json(path, {**copy.deepcopy(self.result), "inputs": copy.deepcopy(self.inputs()),
                          "export_unix_ns": time.time_ns()})
