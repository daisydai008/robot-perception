"""Offline URDF forward kinematics and recorded Tianji state references; no hardware I/O."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zipfile
import zlib

import numpy as np

from .common import require


LIMIT_TOLERANCE = 1e-12  # Radians/metres; numerical roundoff only, not a safety margin.


def _numbers(text, count, label):
    try:
        values = np.array([float(v) for v in text.split()], dtype=float)
    except ValueError:
        require(False, f"{label}: expected {count} finite numbers")
    require(values.shape == (count,) and np.isfinite(values).all(),
            f"{label}: expected {count} finite numbers")
    return values


def origin_transform(xyz, rpy):
    """URDF fixed-axis roll/pitch/yaw: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    result = np.eye(4)
    result[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]]
    result[:3, 3] = xyz
    return result


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None

    @property
    def origin(self):
        return origin_transform(self.xyz, self.rpy)

    def transform(self, value=0.0):
        require(math.isfinite(value), f"{self.name}: joint value must be finite")
        if self.kind in ("revolute", "prismatic"):
            require(self.lower - LIMIT_TOLERANCE <= value <= self.upper + LIMIT_TOLERANCE,
                    f"{self.name}: {value} outside limits [{self.lower}, {self.upper}]")
        motion = np.eye(4)
        if self.kind in ("revolute", "continuous"):
            x, y, z = self.axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            motion[:3, :3] = (np.eye(3) + math.sin(value) * skew
                             + (1 - math.cos(value)) * (skew @ skew))
        elif self.kind == "prismatic":
            motion[:3, 3] = self.axis * value
        return self.origin @ motion


@dataclass
class URDFModel:
    robot_name: str
    links: set[str]
    joints: dict[str, Joint]
    parents: dict[str, Joint]
    urdf_sha256: str
    urdf_path: str | None = None
    urdf_member: str | None = None

    def chain(self, base, target):
        require(base in self.links, f"Missing base link: {base}")
        require(target in self.links, f"Missing target link: {target}")
        result = []
        link = target
        while link != base:
            require(link in self.parents, f"Target {target} is not under base {base}")
            joint = self.parents[link]
            result.append(joint)
            link = joint.parent
        return list(reversed(result))

    def fk(self, base, target, positions):
        """Return T_base_target in metres; positions maps movable names to rad/metres."""
        chain = self.chain(base, target)
        names = {joint.name for joint in chain if joint.kind != "fixed"}
        require(set(positions) == names, "Joint positions must exactly match the movable chain joint names")
        result = np.eye(4)
        for joint in chain:
            value = positions[joint.name] if joint.kind != "fixed" else 0.0
            require(isinstance(value, (int, float, np.integer, np.floating))
                    and not isinstance(value, (bool, np.bool_)) and math.isfinite(value),
                    f"{joint.name}: joint value must be a finite number")
            result = result @ joint.transform(value)
        require(np.isfinite(result).all(), "Non-finite FK result")
        return result


def parse_urdf(xml_bytes):
    """Parse XML bytes without meshes or ROS; reject ambiguous/non-tree topology."""
    try:
        root = ET.fromstring(xml_bytes)
    except (ET.ParseError, LookupError, UnicodeError) as exc:
        require(False, f"Invalid URDF XML: {exc}")
    require(root.tag == "robot", "URDF root must be robot")
    names = [element.get("name") for element in root.findall("link")]
    require(names and all(names) and len(set(names)) == len(names), "Missing or duplicate URDF link names")
    links, joints, parents = set(names), {}, {}
    for element in root.findall("joint"):
        name, kind = element.get("name"), element.get("type")
        require(name and name not in joints, "Missing or duplicate URDF joint name")
        require(kind in ("fixed", "revolute", "continuous", "prismatic"),
                f"{name}: unsupported joint type {kind}")
        parent, child = element.find("parent"), element.find("child")
        require(parent is not None and child is not None, f"{name}: missing parent/child")
        parent, child = parent.get("link"), child.get("link")
        require(parent in links and child in links, f"{name}: missing referenced link")
        require(child not in parents, f"{child}: multiple parent joints")
        origin = element.find("origin")
        xyz = _numbers(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3, f"{name} xyz")
        rpy = _numbers(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3, f"{name} rpy")
        axis_element = element.find("axis")
        axis = _numbers(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0",
                        3, f"{name} axis")
        # Scale first to avoid overflow/underflow while normalising finite axes.
        scale = np.max(np.abs(axis))
        require(kind == "fixed" or scale > 0, f"{name}: movable joint axis must be nonzero")
        if scale > 0:
            axis = axis / scale
            axis = axis / np.linalg.norm(axis)
        # Fixed joints do not use an axis; vendor URDFs may explicitly use 0 0 0.
        lower = upper = None
        if kind in ("revolute", "prismatic"):
            limit = element.find("limit")
            require(limit is not None and "lower" in limit.attrib and "upper" in limit.attrib,
                    f"{name}: lower/upper limits required")
            lower, upper = _numbers(f"{limit.get('lower')} {limit.get('upper')}", 2, f"{name} limits")
            require(lower <= upper, f"{name}: lower limit exceeds upper")
        joint = Joint(name, kind, parent, child, xyz, rpy, axis, lower, upper)
        joints[name] = parents[child] = joint
    # Check the entire graph, including branches not used by the requested chain.
    visited = set()
    for link in links:
        path = set()
        while link not in visited:
            require(link not in path, f"URDF link cycle at {link}")
            path.add(link)
            if link not in parents:
                break
            link = parents[link].parent
        visited.update(path)
    return URDFModel(root.get("name", ""), links, joints, parents, hashlib.sha256(xml_bytes).hexdigest())


def load_urdf(path, member=None):
    """Read .urdf or a zip member in place; hash the XML bytes, not the archive."""
    path = Path(path)
    require(path.suffix.lower() in (".urdf", ".zip"), "URDF path must end in .urdf or .zip")
    if path.suffix.lower() == ".zip":
        try:
            with zipfile.ZipFile(path) as archive:
                choices = [info for info in archive.infolist()
                           if not info.is_dir() and info.filename.lower().endswith(".urdf")]
                if member is None:
                    require(len(choices) == 1, "Zip must contain exactly one .urdf, or specify --urdf-member")
                    member = choices[0].filename
                matches = [info for info in choices if info.filename == member]
                require(len(matches) == 1, f"Missing or ambiguous URDF zip member: {member}")
                xml_bytes = archive.read(matches[0])
        except (zipfile.BadZipFile, RuntimeError, NotImplementedError, zlib.error) as exc:
            require(False, f"Cannot read URDF zip: {exc}")
    else:
        require(member is None, "--urdf-member is only valid with a .zip")
        xml_bytes = path.read_bytes()
    model = parse_urdf(xml_bytes)
    model.urdf_path, model.urdf_member = str(path.resolve()), member
    return model


_FIELD = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(\[[^\]\r\n]*\]|[^\s]+)")
_NUMBER = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
_SAMPLE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,199}")


def parse_state_line(line, arm):
    """Read pasted CLI/control-session key=value fields, never import the SDK."""
    require(arm in ("left", "right"), "Arm must be left or right")
    fields, end = {}, 0
    for match in _FIELD.finditer(line):
        require((match.start() == 0 or line[match.start() - 1].isspace())
                and not line[end:match.start()].strip(), "Malformed state line")
        key, value = match.groups()
        require(key not in fields, f"Duplicate state field: {key}")
        fields[key] = value
        end = match.end()
    require(not line[end:].strip(), "Malformed state line")
    require(fields.get("arm") == arm, f"State arm must match --arm {arm}")
    raw = fields.get("position_degrees", "")
    require(raw.startswith("[") and raw.endswith("]"), "Missing/malformed position_degrees list")
    values = [value.strip() for value in raw[1:-1].split(",")]
    require(len(values) == 7 and all(_NUMBER.fullmatch(value) for value in values),
            "position_degrees must contain exactly 7 finite numbers")
    degrees = [float(value) for value in values]
    require(all(math.isfinite(value) for value in degrees), "position_degrees must be finite")
    if "error_raw" in fields:
        require(re.fullmatch(r"[+-]?[0-9]+", fields["error_raw"]) is not None
                and int(fields["error_raw"]) == 0, "State error_raw must be 0")
    if "frame" in fields:
        require(re.fullmatch(r"[+-]?[0-9]+", fields["frame"]) is not None, "State frame must be an integer")
    return {"position_degrees": degrees, "frame": fields.get("frame", "n/a"), "state_line": line}


def parse_samples(text, arm):
    samples, ids = [], set()
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = line.lstrip().split(maxsplit=1)
        require(len(parts) == 2, f"Sample line {number}: expected sample_id and state line")
        sample_id, state_line = parts
        require(_SAMPLE_ID.fullmatch(sample_id) is not None, f"Sample line {number}: unsafe sample ID")
        require(sample_id not in ids, f"Duplicate sample ID: {sample_id}")
        ids.add(sample_id)
        samples.append({"sample_id": sample_id, **parse_state_line(state_line, arm)})
    require(samples, "Samples file has no recorded states")
    return samples


def write_ee_references(args):
    """Validate the entire batch before creating any output; never overwrite files."""
    require(args.confirm_settled_same_pose,
            "ee-reference requires --confirm-settled-same-pose for every frozen image/state pair")
    model = load_urdf(args.urdf, args.urdf_member)
    chain = [joint for joint in model.chain(args.base, args.ee) if joint.kind != "fixed"]
    require(len(chain) == 7, "base->ee chain must contain exactly 7 movable joints")
    require(all(joint.kind in ("revolute", "continuous") for joint in chain),
            "ee-reference requires angular joints; position_degrees cannot supply prismatic metres")
    samples = parse_samples(Path(args.samples).read_text(encoding="utf-8"), args.arm)
    out_dir = Path(args.out_dir)
    require(not out_dir.exists() or out_dir.is_dir(), f"Not an output directory: {out_dir}")
    pending = []
    joint_names = [joint.name for joint in chain]
    for sample in samples:
        path = out_dir / f"{sample['sample_id']}.json"
        require(not path.exists() and not path.is_symlink(), f"Refusing existing output: {path}")
        degrees = sample["position_degrees"]
        matrix = model.fk(args.base, args.ee, dict(zip(joint_names, np.deg2rad(degrees))))
        reference = {
            "units": "m", "from_frame": args.ee, "to_frame": args.base, "matrix": matrix.tolist(),
            "source_record": (f"{sample['sample_id']}: tianji state frame={sample['frame']} "
                              f"position_degrees={degrees}; URDF {model.urdf_member or model.urdf_path} "
                              f"sha256={model.urdf_sha256}"),
            "settled_same_pose": True,
            "provenance": {"tool": "robot_perception.calibration.urdf_fk", "urdf_path": model.urdf_path,
                           "urdf_member": model.urdf_member, "urdf_sha256": model.urdf_sha256,
                           "urdf_robot_name": model.robot_name, "base": args.base, "ee": args.ee,
                           "arm": args.arm, "joint_names": joint_names, "position_degrees": degrees,
                           "state_line": sample["state_line"],
                           "status": "fk_from_vendor_urdf_not_validated_against_robot"}}
        pending.append((path, json.dumps(reference, indent=2, ensure_ascii=False, allow_nan=False) + "\n"))
    out_dir.mkdir(parents=True, exist_ok=True)
    for path, text in pending:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(text)
    return len(pending)
