"""Fixed-camera geometry only. T_A_B maps B coordinates into A; lengths are metres."""
import copy
import hashlib
import json

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .common import require

ORIGIN = "first_inner_corner; z=0_printed_surface; no_thickness_offset"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                     separators=(",", ":")).encode()).hexdigest()


def defaults():
    return {"mode": "known_board", "fixed_camera": False,
            "frames": {"base": "base", "color": "d435_color_optical_frame",
                       "board": "board", "ee": "end_effector"},
            "board": {"squares_x": 12, "squares_y": 9, "square_m": 0.020,
                      "pattern_mm": [240, 180], "plate_mm": [290, 230], "thickness_mm": 3.2,
                      "origin": ORIGIN, "scale_confirmed": False, "measurement_record": ""},
            "raw_model_confirmed": False, "profile_record": "",
            "thresholds": {"translation_mm": None, "rotation_deg": None, "reprojection_px": None,
                           "confirmed": False, "record": ""}}


def finite_array(value, shape, name):
    a = np.asarray(value, dtype=float)
    require(a.shape == shape and np.isfinite(a).all(), f"{name}: invalid shape / nonfinite")
    return a


def se3(value):
    a = finite_array(value, (4, 4), "SE3")
    r = a[:3, :3]
    require(np.allclose(a[3], [0, 0, 0, 1], atol=1e-8, rtol=0), "SE3: bottom row")
    require(np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0) and
            abs(np.linalg.det(r) - 1) < 1e-6, "SE3: rotation must be SO(3), det +1")
    return a


def transform(rotvec, translation):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    t[:3, 3] = translation
    return t


def inverse(t):
    t = se3(t)
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def validate_config(config, metric=False):
    require(config["mode"] in ("known_board", "eye_to_hand"), "Unsupported mode")
    f, b = config["frames"], config["board"]
    require(set(f) == {"base", "color", "board", "ee"} and
            all(isinstance(x, str) and x.strip() == x and x and not any(c.isspace() for c in x)
                for x in f.values()) and len(set(f.values())) == 4, "Four distinct frame names required")
    require(all(type(b[k]) is int and 3 <= b[k] <= 40 for k in ("squares_x", "squares_y")),
            "Board dimensions are SQUARE counts (3..40)")
    require(type(b["square_m"]) in (float, int) and np.isfinite(b["square_m"]) and b["square_m"] > 0,
            "Square length must be positive metres")
    require(b["origin"] == ORIGIN, "Board origin must be first INNER corner on printed surface")
    for k in ("fixed_camera", "raw_model_confirmed"):
        require(type(config[k]) is bool, f"{k} must be boolean")
    require(type(b["scale_confirmed"]) is bool, "scale_confirmed must be boolean")
    if metric:
        require(config["fixed_camera"], "Confirm fixed camera/head/base configuration; moving camera unsupported")
        require(config["raw_model_confirmed"] and bool(config["profile_record"].strip()),
                "Confirm full-resolution raw pinhole plumb_bob semantics and profile record")
        require(b["scale_confirmed"] and bool(b["measurement_record"].strip()),
                "Independently measured square scale / record not confirmed")
        th = config["thresholds"]
        require(th["confirmed"] is True and bool(th["record"].strip()), "Threshold approval record required")
        require(all(type(th[k]) in (float, int) and np.isfinite(th[k]) and th[k] > 0
                    for k in ("translation_mm", "rotation_deg", "reprojection_px")),
                "Positive thresholds required: mm, deg, px; no implicit pass defaults")
    return config


def camera_info(info):
    """Strict raw full-frame Brown5 subset, not a generic CameraInfo interpreter."""
    require(isinstance(info, dict), "CameraInfo must be an object")
    require(all(type(info.get(k)) is int and 0 < info[k] <= 16384 for k in ("width", "height")),
            "Invalid CameraInfo size")
    require(info.get("distortion_model") == "plumb_bob", "Only confirmed raw pinhole plumb_bob supported")
    k = finite_array(info["k"], (9,), "K").reshape(3, 3)
    d = finite_array(info["d"], (5,), "D")
    r = finite_array(info["r"], (9,), "R").reshape(3, 3)
    p = finite_array(info["p"], (12,), "P").reshape(3, 4)
    require(k[0, 0] > 0 and k[1, 1] > 0 and 0 <= k[0, 2] < info["width"] and
            0 <= k[1, 2] < info["height"] and np.allclose(k[2], [0, 0, 1]) and
            k[0, 1] == 0 and k[1, 0] == 0, "Invalid pinhole K")
    require(np.allclose(r, np.eye(3), atol=1e-8, rtol=0) and
            np.allclose(p[:, :3], k, atol=1e-6, rtol=0) and np.all(p[:, 3] == 0),
            "Rectification/projection ambiguity: require R=I, P=[K|0]; confirm raw image separately")
    require(info.get("binning_x") in (0, 1) and info.get("binning_y") in (0, 1),
            "Missing/unsupported binning")
    roi = info.get("roi")
    require(isinstance(roi, dict) and set(roi) == {"x_offset", "y_offset", "height", "width", "do_rectify"}
            and all(v == 0 for v in roi.values()), "Missing/nonzero ROI or rectify flag")
    validate_header(info["header"])
    return k, d


def validate_header(h):
    require(isinstance(h, dict) and isinstance(h.get("frame_id"), str) and bool(h["frame_id"].strip()),
            "Missing frame_id")
    require(type(h.get("sec")) is int and h["sec"] >= 0 and type(h.get("nanosec")) is int and
            0 <= h["nanosec"] < 10**9 and h["sec"] * 10**9 + h["nanosec"] > 0, "Invalid source stamp")
    return h["sec"] * 10**9 + h["nanosec"]


def profile(info):
    camera_info(info)
    return {k: copy.deepcopy(info[k]) for k in ("width", "height", "distortion_model", "k", "d", "r", "p",
                                               "binning_x", "binning_y", "roi")} | {"frame_id": info["header"]["frame_id"]}


def object_points(board):
    x, y = board["squares_x"] - 1, board["squares_y"] - 1
    points = np.zeros((x * y, 3), np.float64)
    points[:, :2] = np.mgrid[0:x, 0:y].T.reshape(-1, 2) * board["square_m"]
    return points


def project(t, obj, k, d):
    t = se3(t)
    require(np.all((obj @ t[:3, :3].T + t[:3, 3])[:, 2] > 0), "Board behind camera")
    return cv2.projectPoints(obj, cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], k, d)[0].reshape(-1, 2)


def reprojection(t, obj, corners, k, d):
    err = np.linalg.norm(project(t, obj, k, d) - corners, axis=1)
    return {"rms_px": float(np.sqrt(np.mean(err ** 2))), "max_px": float(err.max())}


def planar_candidates(corners, info, board):
    require(board["scale_confirmed"] is True and bool(board["measurement_record"].strip()),
            "Confirm independently measured scale before metric PnP")
    k, d = camera_info(info)
    obj = object_points(board)
    corners = finite_array(corners, (len(obj), 2), "corners")
    require(np.linalg.matrix_rank(corners - corners.mean(axis=0)) == 2, "Collinear image points")
    out = cv2.solvePnPGeneric(obj, corners, k, d, flags=cv2.SOLVEPNP_IPPE)
    require(out[0] > 0, "IPPE failed")
    candidates = []
    for rvec, tvec in zip(out[1], out[2]):
        t = transform(np.asarray(rvec).ravel(), np.asarray(tvec).ravel())
        positive = bool(np.all((obj @ t[:3, :3].T + t[:3, 3])[:, 2] > 0))
        metrics = reprojection(t, obj, corners, k, d) if positive else {"rms_px": None, "max_px": None}
        candidates.append({"T_C_Q": t.tolist(), "positive_depth": positive, **metrics})
    return candidates


def detect(image, info, board, reverse=False, raw_model_confirmed=False):
    camera_info(info)
    require(image is not None and image.shape == (info["height"], info["width"], 3), "Image/CameraInfo dimensions differ")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    size = (board["squares_x"] - 1, board["squares_y"] - 1)
    found, corners = cv2.findChessboardCornersSB(
        gray, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    method = "findChessboardCornersSB"
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        method = "findChessboardCorners+cornerSubPix"
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    if not found:
        return {"corners": [], "candidates": [], "reverse": reverse, "status": "未找到完整棋盘"}
    corners = corners.reshape(-1, 2).astype(float)
    if reverse:
        corners = corners[::-1].copy()
    confirmed = (board["scale_confirmed"] is True and bool(board["measurement_record"].strip())
                 and raw_model_confirmed is True)
    candidates = planar_candidates(corners, info, board) if confirmed else []
    return {"corners": corners.tolist(), "candidates": candidates, "reverse": reverse,
            "detector": method,
            "status": ("IPPE 平面候选：必须核对物理方向并选择；低残差不消除歧义" if confirmed else
                       "仅角点预览：尺度或 raw 模型未确认，不计算米制 PnP；确认后重新导入/采集")}


def validate_reference(ref, config):
    require(isinstance(ref, dict), "Missing external reference; preview only")
    require(ref.get("units") == "m", "External reference units must be m")
    source = "board" if config["mode"] == "known_board" else "ee"
    require(ref.get("from_frame") == config["frames"][source] and ref.get("to_frame") == config["frames"]["base"],
            "Reference direction/frame mismatch (must map board/ee into base)")
    require(isinstance(ref.get("source_record"), str) and bool(ref["source_record"].strip()), "Reference source record required")
    require(ref.get("settled_same_pose") is True, "Confirm settled image and external reference at SAME pose")
    return se3(ref["matrix"])


def mean_transform(ts):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_matrix(np.array([a[:3, :3] for a in ts])).mean().as_matrix()
    t[:3, 3] = np.mean([a[:3, 3] for a in ts], axis=0)
    return t


def unique_poses(ts):
    for i, a in enumerate(ts):
        for b in ts[:i]:
            delta = inverse(a) @ b
            require(np.linalg.norm(delta[:3, 3]) > 1e-5 or
                    np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()) > 1e-4,
                    "Duplicate/identical poses: collect independent views, not adjacent static frames")


def excitation(ts):
    """Numerical observability guard, NOT physical acceptance thresholds."""
    relative = [inverse(ts[0]) @ t for t in ts[1:]]
    rot = np.array([Rotation.from_matrix(t[:3, :3]).as_rotvec() for t in relative])
    trans = np.array([t[:3, 3] for t in relative])
    sr = np.linalg.svd(rot, compute_uv=False)
    st = np.linalg.svd(trans, compute_uv=False)
    require(sr[1] > 1e-3 and sr[1] / sr[0] > 1e-3, "Single-axis/insufficient rotational excitation")
    require(st[1] > 1e-4 and st[1] / st[0] > 1e-3, "Insufficient multi-axis translation excitation")
    return {"rotation_singular_values_rad": sr.tolist(), "translation_singular_values_m": st.tolist()}


def fit_handeye(a, c):
    # OpenCV solves A * base2world = gripper2cam * B. Here A=T_B_E, B=T_C_Q;
    # therefore base2world=Y=T_E_Q, gripper2cam=X=T_B_C. Verified by synthetic tests.
    ry, ty, rx, tx = cv2.calibrateRobotWorldHandEye(
        [t[:3, :3] for t in a], [t[:3, 3] for t in a],
        [t[:3, :3] for t in c], [t[:3, 3] for t in c], method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    initial = np.r_[Rotation.from_matrix(rx).as_rotvec(), np.ravel(tx),
                    Rotation.from_matrix(ry).as_rotvec(), np.ravel(ty)]

    def unpack(p):
        return transform(p[:3], p[3:6]), transform(p[6:9], p[9:12])

    def residual(p):
        x, y = unpack(p)
        rows = []
        for ai, ci in zip(a, c):
            delta = inverse(ai @ y) @ x @ ci
            rows.extend(delta[:3, 3])
            rows.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec())
        return rows

    fit = least_squares(residual, initial, max_nfev=300, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    require(fit.success and np.isfinite(fit.x).all(), "Handeye least-squares did not converge")
    spectrum = np.linalg.svd(fit.jac, compute_uv=False)
    require(spectrum[-1] / spectrum[0] > 1e-6, "Handeye rank deficient")
    return (*unpack(fit.x), spectrum.tolist())


def solve_inputs(config, samples):
    """Check readiness without freezing a session or fitting any heldout data."""
    validate_config(config, metric=True)
    active = [s for s in samples if not s["excluded"]]
    train = [s for s in active if s["split"] == "train"]
    heldout = [s for s in active if s["split"] == "heldout"]
    limits = (3, 2) if config["mode"] == "known_board" else (6, 3)
    require(len(train) >= limits[0] and len(heldout) >= limits[1],
            f"Need >= {limits[0]} train + {limits[1]} heldout independent poses; minima are not acceptance")
    refs, poses = {}, {}
    for s in active:
        require(s["orientation_confirmed"] is True and s["candidate_confirmed"] is True,
                f"{s['id']}: confirm physical corner0/+x/+y and planar candidate")
        require(type(s["selected"]) is int and 0 <= s["selected"] < len(s["candidates"]), "Select a candidate")
        candidate = s["candidates"][s["selected"]]
        require(candidate["positive_depth"] is True, "Candidate not positive depth")
        refs[s["id"]] = validate_reference(s["reference"], config)
        poses[s["id"]] = se3(candidate["T_C_Q"])
    require(len({s["reference"]["source_record"] for s in active}) == len(active),
            "Each reference needs its own source record; no automatic reuse")
    unique_poses(list(refs.values()))
    unique_poses(list(poses.values()))
    a, c = [refs[s["id"]] for s in train], [poses[s["id"]] for s in train]
    if config["mode"] == "eye_to_hand":
        excitation(a)
        excitation(c)
    return active, train, refs, poses


def solve(config, samples):
    active, train, refs, poses = solve_inputs(config, samples)
    a, c = [refs[s["id"]] for s in train], [poses[s["id"]] for s in train]
    y, diagnostics = None, {}
    if config["mode"] == "known_board":
        x = mean_transform([ai @ inverse(ci) for ai, ci in zip(a, c)])
    else:
        diagnostics["reference_excitation"] = excitation(a)
        diagnostics["camera_excitation"] = excitation(c)
        x, y, diagnostics["jacobian_singular_values"] = fit_handeye(a, c)
    metrics = []
    th = config["thresholds"]
    obj = object_points(config["board"])
    for s in active:
        left = refs[s["id"]] if y is None else refs[s["id"]] @ y
        delta = inverse(left) @ x @ poses[s["id"]]
        m = {"id": s["id"], "split": s["split"],
             "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
             "rotation_deg": float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))),
             "pnp_rms_px": s["candidates"][s["selected"]]["rms_px"], "failures": []}
        try:
            k, d = camera_info(s["camera_info"])
            m.update(reprojection(inverse(x) @ left, obj, np.asarray(s["corners"]), k, d))
        except ValueError as exc:
            m.update(rms_px=None, max_px=None)
            m["failures"].append(str(exc))
        for field, threshold in (("translation_mm", "translation_mm"), ("rotation_deg", "rotation_deg"),
                                 ("rms_px", "reprojection_px"), ("pnp_rms_px", "reprojection_px")):
            if m[field] is None or m[field] > th[threshold]:
                m["failures"].append(f"{field} exceeds {th[threshold]}")
        m["pass"] = not m["failures"]
        metrics.append(m)
    passed = all(m["pass"] for m in metrics)
    return {"status": "candidate_not_approved" if passed else "failed_diagnostic_not_approved",
            "units": "m", "rotation_representation": "SO3 matrix", "T_B_C": se3(x).tolist(),
            "from_frame": config["frames"]["color"], "to_frame": config["frames"]["base"],
            "T_E_Q": None if y is None else se3(y).tolist(), "metrics": metrics,
            "threshold_pass": passed, "diagnostics": diagnostics,
            "interpretation": "Fixed camera/head/base only. Train fit; heldout evaluated with fixed X,Y. "
                              "Numerical candidate, NEVER hardware acceptance. No automatic outlier removal.",
            "opencv_version": cv2.__version__}
