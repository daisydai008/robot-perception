"""Shared offline validation; importing this module does not import ROS or OpenCV."""
import hashlib
import json
import math
from pathlib import Path


class CalibrationError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise CalibrationError(message)


def positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def read_json(path):
    def reject(value):
        raise CalibrationError(f"Non-finite JSON constant: {value}")
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    # Serialize before opening; exclusive creation never truncates an existing result.
    text = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(text)


def provenance(path):
    return {"path": str(Path(path).resolve()), "sha256": sha256(path), "config": read_json(path)}


def config_errors(config, target="all", protocol=False):
    errors = []
    if not isinstance(config, dict):
        return ["config must be a JSON object"]

    def check(ok, text):
        if not ok:
            errors.append(text)

    check(type(config.get("schema_version")) is int and config["schema_version"] == 1,
          "schema_version must be 1")
    check(bool(config.get("measurement_record")) and isinstance(config.get("measurement_record"), str),
          "measurement_record: supply the actual measurement record reference")
    if target in ("all", "board"):
        b = config.get("board")
        if not isinstance(b, dict):
            errors.append("board must be an object")
        else:
            check(b.get("delivery_confirmed") is True, "board.delivery_confirmed must be true after physical inspection")
            check(b.get("dictionary") == "DICT_6X6_250", "board.dictionary must be DICT_6X6_250 (only supported board dictionary)")
            check(type(b.get("legacy_pattern")) is bool, "board.legacy_pattern must explicitly match the supplier's generation convention")
            dims = [b.get("squares_x"), b.get("squares_y")]
            dims_ok = all(type(n) is int and 3 <= n <= 30 for n in dims)
            check(dims_ok, "board.squares_x/y must be integers from 3 to 30, not inner-corner counts")
            ids = b.get("marker_ids")
            ids_ok = isinstance(ids, list) and all(type(n) is int and 0 <= n < 250 for n in ids)
            check(ids_ok, "board.marker_ids must be an explicit ordered list of IDs in [0,249]")
            if ids_ok:
                check(len(set(ids)) == len(ids), "board.marker_ids contains duplicates")
                if dims_ok:
                    check(len(ids) == dims[0] * dims[1] // 2, "board.marker_ids count does not match board square layout")
            s, m = b.get("square_length_m"), b.get("marker_length_m")
            check(positive(s), "board.square_length_m requires a positive finite measured value in metres")
            check(positive(m), "board.marker_length_m requires a positive finite measured value in metres")
            if positive(s) and positive(m):
                check(m < s, "board.marker_length_m must be smaller than square_length_m")
    if target in ("all", "tags"):
        tags = config.get("apriltags")
        if not isinstance(tags, dict):
            errors.append("apriltags must be an object")
        else:
            check(tags.get("delivery_confirmed") is True, "apriltags.delivery_confirmed must be true after physical inspection")
            check(tags.get("dictionary") == "DICT_APRILTAG_36h11", "apriltags.dictionary must be DICT_APRILTAG_36h11")
            check(tags.get("size_definition") == "black_outer_border", "apriltags.size_definition must be black_outer_border")
            sizes = tags.get("side_lengths_m")
            if not isinstance(sizes, dict) or not sizes:
                errors.append("apriltags.side_lengths_m requires ID-to-measured-side entries")
            else:
                for key, size in sizes.items():
                    check(key.isdigit() and str(int(key)) == key and 0 <= int(key) < 587,
                          f"apriltags ID {key!r} must be a canonical integer string in [0,586]")
                    check(positive(size), f"apriltags.side_lengths_m.{key} requires a measured black-border side in metres")
    if protocol:
        p = config.get("protocol", {})
        if not isinstance(p, dict):
            return errors + ["protocol must be an object"]
        for key in ("approved_by", "frozen_on", "record"):
            check(isinstance(p.get(key), str) and bool(p[key].strip()), f"protocol.{key} must identify the pre-collection approval")
        limits = p.get("limits", {})
        if not isinstance(limits, dict):
            return errors + ["protocol.limits must be an object"]
        for key in ("rgb_depth_pair_delta_ms", "intrinsics_heldout_rms_px", "independent_length_error_m",
                    "extrinsic_translation_error_m", "extrinsic_rotation_error_deg", "image_robot_time_error_ms"):
            check(positive(limits.get(key)), f"protocol.limits.{key} must be frozen before formal capture")
    return errors


def load_config(path, target="all", protocol=False):
    config = read_json(path)
    errors = config_errors(config, target, protocol)
    require(not errors, "Configuration blocked:\n- " + "\n- ".join(errors))
    return config


def vision():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise CalibrationError("OpenCV contrib/NumPy unavailable; use PYTHONNOUSERSITE=1 /usr/bin/python3 on the documented host") from exc
    require(hasattr(cv2, "aruco"), "OpenCV contrib with aruco is required")
    return cv2, np


def make_board(config):
    cv2, np = vision()
    b = config["board"]
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    ids = np.array(b["marker_ids"], dtype=np.int32)
    args = (b["square_length_m"], b["marker_length_m"], dictionary)
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard((b["squares_x"], b["squares_y"]), *args, ids)
    else:
        require(hasattr(cv2.aruco, "CharucoBoard_create"), "This OpenCV build has no ChArUco board API")
        board = cv2.aruco.CharucoBoard_create(b["squares_x"], b["squares_y"], *args)
        board.ids = ids  # Old API otherwise silently starts at ID zero.
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(b["legacy_pattern"])
    else:
        version = tuple(int(x) for x in cv2.__version__.split(".")[:2])
        default_legacy = version < (4, 6)
        require(b["legacy_pattern"] == default_legacy,
                f"OpenCV {cv2.__version__} cannot represent the requested legacy_pattern={b['legacy_pattern']}; "
                "use a compatible environment, do not change the physical-pattern declaration")
    return board, dictionary


def board_points(board):
    return board.getChessboardCorners() if hasattr(board, "getChessboardCorners") else board.chessboardCorners


def detect_markers(gray, dictionary):
    cv2, _ = vision()
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary)


def detect_board(gray, board, dictionary):
    cv2, _ = vision()
    corners, ids, _ = detect_markers(gray, dictionary)
    if ids is None:
        return None, None
    _, corners, ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    return corners, ids
