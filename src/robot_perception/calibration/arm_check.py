"""Offline arm/RGB/native-depth diagnostics. T_A_B maps B into A; metres internally."""
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
import re
import struct
import xml.etree.ElementTree as ET
import zipfile
import zlib

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .common import positive, read_json, require, sha256, write_json
from .extrinsics import camera_info, inverse, se3
from .rgbd_snapshot import static_transform
from .urdf_fk import load_urdf, origin_transform, parse_samples

COLORS = {"right": (230, 140, 40), "left": (180, 60, 220)}  # BGR
SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,199}")


def apply(matrix, points):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def exp_se3(xi):
    """SE(3) exponential, xi=[rotation radians, translation tangent metres]."""
    w, v = np.asarray(xi[:3]), np.asarray(xi[3:])
    theta = np.linalg.norm(w)
    x, y, z = w
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    a = .5 - theta ** 2 / 24 if theta < 1e-5 else (1 - np.cos(theta)) / theta ** 2
    b = 1 / 6 - theta ** 2 / 120 if theta < 1e-5 else (theta - np.sin(theta)) / theta ** 3
    out = np.eye(4)
    out[:3, :3] = Rotation.from_rotvec(w).as_matrix()
    out[:3, 3] = (np.eye(3) + a * skew + b * skew @ skew) @ v
    return out


def read_stl(data):
    """Binary length disambiguates STL even when its header starts with 'solid'."""
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50 * count:
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].astype(float)
    else:
        try:
            text = data.decode("ascii")
            vertices = re.findall(r"\bvertex\s+(\S+)\s+(\S+)\s+(\S+)", text)
            require(text.lstrip().startswith("solid") and "endsolid" in text and vertices
                    and len(vertices) % 3 == 0, "Invalid ASCII/binary STL")
            triangles = np.array(vertices, dtype=float).reshape(-1, 3, 3)
        except (UnicodeError, ValueError) as exc:
            require(False, f"Invalid STL: {exc}")
    require(np.isfinite(triangles).all(), "Non-finite STL vertices")
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    triangles = triangles[areas > 1e-16]
    require(len(triangles) > 0, "STL has no nondegenerate triangles")
    return triangles


def numbers(text, label):
    values = np.array([float(x) for x in text.split()], dtype=float)
    require(values.shape == (3,) and np.isfinite(values).all(), f"Invalid {label}: expected three finite values")
    return values


def load_visuals(model, links, mesh_root=None):
    """Read meshes in place; package://pkg/path resolves to a unique */path zip member."""
    path = Path(model.urdf_path)
    require(path.suffix.lower() == ".zip" or mesh_root is not None, ".urdf requires --mesh-root DIR")
    archive = zipfile.ZipFile(path) if path.suffix.lower() == ".zip" else None
    try:
        xml = archive.read(model.urdf_member) if archive else path.read_bytes()
        root = ET.fromstring(xml)
        elements = {element.get("name"): element for element in root.findall("link")}
        result = {}
        for link in links:
            require(link in elements, f"Missing render link: {link}")
            visuals = elements[link].findall("visual")
            require(visuals, f"{link}: no visual mesh")
            meshes = []
            for visual in visuals:
                mesh = visual.find("geometry/mesh")
                require(mesh is not None and bool(mesh.get("filename")), f"{link}: only STL visual meshes supported")
                filename = mesh.get("filename")
                if filename.startswith("package://"):
                    parts = filename[len("package://"):].split("/", 1)
                    require(len(parts) == 2 and all(parts), f"Invalid package mesh URI: {filename}")
                    relative = parts[1]
                else:
                    relative = filename
                relative_path = PurePosixPath(relative)
                require(not relative_path.is_absolute() and ".." not in relative_path.parts
                        and ":" not in relative and relative_path.suffix.lower() == ".stl",
                        f"Mesh must be a relative STL path: {filename}")
                if archive:
                    matches = [info for info in archive.infolist() if not info.is_dir()
                               and info.filename.endswith("/" + relative)]
                    require(len(matches) == 1, f"Missing or ambiguous mesh zip member: {relative}")
                    data = archive.read(matches[0])
                else:
                    mesh_path = (Path(mesh_root) / relative).resolve()
                    require(mesh_path.is_relative_to(Path(mesh_root).resolve()), "Mesh escapes --mesh-root")
                    data = mesh_path.read_bytes()
                triangles = read_stl(data)
                scale = numbers(mesh.get("scale", "1 1 1"), "mesh scale")
                require(np.all(scale > 0), "Mesh scale must be positive")
                origin = visual.find("origin")
                xyz = numbers(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", "visual xyz")
                rpy = numbers(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", "visual rpy")
                meshes.append(apply(origin_transform(xyz, rpy), triangles * scale))
            result[link] = np.concatenate(meshes)
        return result
    except (zipfile.BadZipFile, RuntimeError, NotImplementedError, zlib.error) as exc:
        require(False, f"Cannot read mesh zip: {exc}")
    finally:
        if archive:
            archive.close()


def assignments(entries, label):
    result = {}
    for entry in entries:
        parts = entry.split("=", 1)
        require(len(parts) == 2 and all(parts), f"{label}: expected KEY=VALUE")
        key, value = parts
        require(SAFE_ID.fullmatch(key) is not None, f"{label}: unsafe ID")
        require(key not in result, f"{label}: duplicate {key}")
        result[key] = value
    require(result, f"Missing {label}")
    return result


def arm_chains(model, base, entries, links):
    chains = {}
    for side, end in assignments(entries, "arm-chain").items():
        require(side in COLORS, "arm-chain side must be left or right")
        joints = [j for j in model.chain(base, end) if j.kind != "fixed"]
        require(len(joints) == 7 and all(j.kind in ("revolute", "continuous") for j in joints),
                f"{side}: arm chain must contain exactly 7 angular joints")
        chains[side] = [j.name for j in joints]
    require(len({j for chain in chains.values() for j in chain}) == sum(map(len, chains.values())),
            "Arm chains must not share movable joints (ambiguous arm grouping)")
    owners = {}
    for link in links:
        require(link in model.links, f"Missing render link: {link}")
        joints = {j.name for j in model.chain(base, link) if j.kind != "fixed"}
        owners_for_link = [side for side, chain in chains.items() if joints and joints <= set(chain)]
        require(len(owners_for_link) == 1, f"{link}: every movable joint must be covered by one arm-chain")
        owners[link] = owners_for_link[0]
    return chains, owners


def read_states(path):
    result = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        sides = re.findall(r"(?:^|\s)arm=(left|right)(?=\s|$)", line)
        require(len(sides) == 1, "Each state line must specify exactly one arm=left/right")
        side = sides[0]
        sample = parse_samples(line, side)[0]
        key = (sample["sample_id"], side)
        require(key not in result, f"Duplicate pose/arm state: {key}")
        result[key] = sample
    require(result, "States file has no recorded states")
    return result


def pose_meshes(model, visuals, base, chains, owners, states, pose):
    positions = {}
    for side, joints in chains.items():
        require((pose, side) in states, f"Missing joint state for pose={pose} arm={side}")
        radians = np.deg2rad(states[pose, side]["position_degrees"])
        positions.update(zip(joints, radians))
        for name, value in zip(joints, radians):
            model.joints[name].transform(value)  # Validate even unrendered end joints against limits.
    result = {side: {} for side in set(owners.values())}
    for link, triangles in visuals.items():
        names = [j.name for j in model.chain(base, link) if j.kind != "fixed"]
        matrix = model.fk(base, link, {name: positions[name] for name in names})
        result[owners[link]][link] = apply(matrix, triangles)
    return result


def snapshot_profile(record):
    require(isinstance(record, dict) and all(k in record for k in ("header", "camera_info", "file")),
            "Incomplete snapshot frame record")
    info, hdr = record["camera_info"], record["header"]
    require(isinstance(info, dict) and info.get("header") == hdr, "Snapshot image/CameraInfo headers differ")
    require(isinstance(hdr, dict) and isinstance(hdr.get("stamp"), dict) and "frame_id" in hdr,
            "Invalid snapshot header")
    require(all(k in info for k in ("width", "height", "k", "d", "r", "p", "distortion_model")),
            "Incomplete snapshot CameraInfo")
    flat = {**hdr["stamp"], "frame_id": hdr["frame_id"]}
    k, d = camera_info({**info, "header": flat})
    require(hdr.get("stamp_ns") == flat["sec"] * 10**9 + flat["nanosec"], "Inconsistent snapshot stamp_ns")
    require(np.all(d == 0), "arm-check supports D=0 only; distorted streams are unsupported")
    return k, {key: value for key, value in info.items()
               if key not in ("header", "receipt_unix_ns", "receipt_monotonic_ns")} | {"frame_id": hdr["frame_id"]}


def load_snapshot(directory):
    directory = Path(directory)
    record = read_json(directory / "snapshot.json")
    require(isinstance(record, dict) and record.get("schema_version") == 1
            and record.get("status") == "diagnostic_snapshot_not_calibration", "Unsupported/incomplete snapshot")
    unit = record.get("depth_unit_m")
    require(positive(unit), "Snapshot depth_unit_m must be explicit positive metres per raw unit")
    require(isinstance(record.get("streams"), dict) and isinstance(record.get("tf_static"), list),
            "Snapshot missing streams/static TF")
    arrays, profiles, intrinsics = {}, {}, {}
    for stream in ("color", "depth"):
        records = record["streams"].get(stream)
        require(isinstance(records, list) and 1 <= len(records) <= 300, f"Missing/invalid {stream} frames")
        require(record.get("frames_requested_per_stream") == len(records), "Incomplete snapshot frame count")
        images, stamps = [], set()
        for frame in records:
            k, profile = snapshot_profile(frame)
            require(stream not in profiles or profiles[stream] == profile, f"{stream}: changed CameraInfo profile/frame")
            stamp = frame["header"]["stamp_ns"]
            require(stamp not in stamps, f"{stream}: duplicate snapshot frame timestamp")
            stamps.add(stamp)
            profiles[stream], intrinsics[stream] = profile, k
            filename = frame["file"]
            require(isinstance(filename, str) and Path(filename).name == filename
                    and filename not in (".", ".."), "Snapshot image must be a local filename")
            image_path = directory / filename
            require(image_path.resolve().is_relative_to(directory.resolve()), "Snapshot image escapes directory")
            image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            shape = (profile["height"], profile["width"]) + ((3,) if stream == "color" else ())
            require(image is not None and image.shape == shape and image.dtype ==
                    (np.uint8 if stream == "color" else np.uint16), f"Invalid {stream} PNG/dimensions/encoding")
            require(frame.get("stored_encoding") == ("bgr8" if stream == "color" else "16UC1"),
                    f"Unexpected {stream} stored encoding")
            if stream == "depth" or not images:
                images.append(image)
        arrays[stream] = images
    frames = np.stack(arrays["depth"])
    median = np.zeros(frames.shape[1:], float)
    # Chunk rows to avoid a second full float-sized copy for a 300-frame snapshot.
    for row in range(0, median.shape[0], 32):
        block = frames[:, row:row + 32]
        valid = block > 0
        med = np.ma.median(np.ma.array(block, mask=~valid), axis=0).filled(0)
        med[valid.sum(axis=0) * 2 < len(frames)] = 0
        median[row:row + 32] = med * unit
    color_frame, depth_frame = (profiles[s]["frame_id"] for s in ("color", "depth"))
    require(color_frame != depth_frame, "Expected distinct native depth and color optical frames")
    for tf in record["tf_static"]:
        require(isinstance(tf, dict) and all(k in tf for k in ("parent", "child", "translation", "rotation_xyzw")),
                "Incomplete snapshot static TF")
    return {"record": record, "color": arrays["color"][0], "depth_m": median, "k": intrinsics,
            "color_frame": color_frame, "T_color_depth": static_transform(record["tf_static"], color_frame, depth_frame),
            "path": str(directory.resolve()), "sha256": sha256(directory / "snapshot.json")}


def projected_mask(triangles, T_camera_base, k, shape):
    """Union of projected triangles, clipped at the optical near plane; no hidden-surface claim."""
    mask = np.zeros(shape, np.uint8)
    camera = apply(T_camera_base, triangles)
    near = 1e-5
    front = np.all(camera[:, :, 2] >= near, axis=1)
    polygons = camera[front]
    crossing = camera[~front & np.any(camera[:, :, 2] >= near, axis=1)]
    # Normally no crossing triangles; clipping avoids divisions at/behind the camera.
    extras = []
    for triangle in crossing:
        clipped = []
        for i, point in enumerate(triangle):
            previous = triangle[i - 1]
            if (point[2] >= near) != (previous[2] >= near):
                clipped.append(previous + (point - previous) * ((near - previous[2]) / (point[2] - previous[2])))
            if point[2] >= near:
                clipped.append(point)
        extras.append(np.array(clipped))
    if len(polygons):
        uv = polygons @ k.T
        uv = uv[:, :, :2] / uv[:, :, 2:]
        visible = ((uv[:, :, 0].max(axis=1) >= 0) & (uv[:, :, 0].min(axis=1) < shape[1]) &
                   (uv[:, :, 1].max(axis=1) >= 0) & (uv[:, :, 1].min(axis=1) < shape[0]))
        for polygon in np.rint(np.clip(uv[visible], -1e6, 1e6)).astype(np.int32):
            cv2.fillConvexPoly(mask, polygon, 255)
    for polygon in extras:
        uv = polygon @ k.T
        uv = uv[:, :2] / uv[:, 2:]
        cv2.fillConvexPoly(mask, np.rint(np.clip(uv, -1e6, 1e6)).astype(np.int32), 255)
    return mask


def surface_samples(triangles, spacing):
    """Deterministic area-proportional barycentric samples; winding supplies normals."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    counts = np.maximum(1, np.ceil(lengths / (2 * spacing ** 2)).astype(int))
    index = np.repeat(np.arange(len(triangles)), counts)
    local = np.arange(len(index)) - np.repeat(np.cumsum(counts) - counts, counts)
    # Stratified radial coordinate + irrational angular sequence, deterministic per triangle.
    u = np.sqrt((local + .5) / counts[index])
    v = np.mod((local + .5) * .6180339887498949, 1)
    selected = triangles[index]
    points = ((1 - u[:, None]) * selected[:, 0] + (u * (1 - v))[:, None] * selected[:, 1]
              + (u * v)[:, None] * selected[:, 2])
    return points, normals[index] / lengths[index, None]


@dataclass
class Group:
    pose: str
    arm: str
    color_points: np.ndarray
    pixels: np.ndarray
    roi_pixels: int
    samples: np.ndarray
    normals: np.ndarray
    tree: cKDTree
    sample_counts: dict


def make_groups(pose, snapshot, meshes, camera, dilation, spacing, max_depth):
    depth = snapshot["depth_m"]
    k = snapshot["k"]["depth"]
    T_color_depth = snapshot["T_color_depth"]
    T_depth_base = inverse(camera @ T_color_depth)
    groups = []
    for arm, links in sorted(meshes.items()):
        triangles = np.concatenate(list(links.values()))
        mask = projected_mask(triangles, T_depth_base, k, depth.shape)
        if dilation:
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation + 1,) * 2))
        y, x = np.nonzero((mask > 0) & (depth > 0) & (depth <= max_depth))
        z = depth[y, x]
        native = np.column_stack(((x - k[0, 2]) * z / k[0, 0], (y - k[1, 2]) * z / k[1, 1], z))
        samples, normals, counts = [], [], {}
        for link, surface in links.items():
            points, directions = surface_samples(surface, spacing)
            samples.append(points)
            normals.append(directions)
            counts[link] = len(points)
        samples, normals = np.concatenate(samples), np.concatenate(normals)
        groups.append(Group(pose, arm, apply(T_color_depth, native), np.column_stack((y, x)),
                            int(np.count_nonzero(mask)), samples, normals, cKDTree(samples), counts))
    return groups


def distances(group, camera):
    points = apply(camera, group.color_points)
    d, indices = group.tree.query(points)
    normals = group.normals[indices]
    signed = np.sum((points - group.samples[indices]) * normals, axis=1)
    return d, signed, indices


def group_stats(group, camera, gates, min_points):
    d, signed, _ = distances(group, camera)
    keep = d <= gates[-1]
    usable = int(np.count_nonzero(d <= gates[0])) >= min_points
    return {"pose": group.pose, "arm": group.arm, "roi_pixels": group.roi_pixels,
            "valid_depth_count": len(d), "surface_samples_per_link": group.sample_counts,
            "usable_for_fit": usable,
            "inlier_counts_per_gate": [{"gate_mm": gate * 1000, "count": int(np.count_nonzero(d <= gate))}
                                       for gate in gates],
            "unsigned_distance_mm": dict(zip(("median", "p90", "p95"),
                                             np.percentile(d[keep] * 1000, [50, 90, 95]).tolist())) if keep.any() else None,
            "signed_point_to_plane_mean_mm": float(np.mean(signed[keep]) * 1000) if keep.any() else None}


def correspondences(groups, camera, gate):
    sources, targets, normals = [], [], []
    for group in groups:
        d, _, indices = distances(group, camera)
        keep = d <= gate
        sources.append(group.color_points[keep])
        targets.append(group.samples[indices[keep]])
        normals.append(group.normals[indices[keep]])
    return tuple(np.concatenate(items) for items in (sources, targets, normals))


def observability(source, target, normals, camera):
    """Local camera-side Jacobian; translation columns scaled by one-metre reference length."""
    if len(source) < 6:
        return {"normalised_eigenvalues": [0.] * 6, "weak_directions": 6, "reference_length_m": 1.0}
    local_normals = normals @ camera[:3, :3]
    jacobian = np.column_stack((np.cross(source, local_normals), local_normals))
    residual = np.sum((apply(camera, source) - target) * normals, axis=1)
    # Same soft_l1 curvature weighting as least_squares' effective J.
    weight = (1 + (residual / .005) ** 2) ** (-.75)
    jacobian *= weight[:, None]
    eigenvalues = np.maximum(0, np.linalg.eigvalsh(jacobian.T @ jacobian))
    normalised = eigenvalues / eigenvalues[-1] if eigenvalues[-1] > 0 else np.zeros(6)
    return {"normalised_eigenvalues": normalised.tolist(), "weak_directions": int(np.count_nonzero(normalised < 1e-4)),
            "weak_eigenvalue_ratio_threshold": 1e-4, "reference_length_m": 1.0,
            "jacobian_coordinates": "local camera-side rotation rad, translation / 1 metre; soft_l1 weighted"}


def refine(groups, camera, gates, min_points):
    usable = [g for g in groups if np.count_nonzero(distances(g, camera)[0] <= gates[0]) >= min_points]
    xi, history = np.zeros(6), []
    if not usable:
        return camera.copy(), {"status": "insufficient_points", "xi_rotation_deg": [0.] * 3,
                               "xi_translation_mm": [0.] * 3, "translation_norm_mm": 0., "rotation_norm_deg": 0.,
                               "corrected_T_base_color": camera.tolist(), "iterations": [],
                               "observability": observability(np.empty((0, 3)), None, None, camera)}
    status = "converged_schedule"
    for gate in gates:
        converged = False
        for iteration in range(15):
            source, target, normals = correspondences(usable, camera @ exp_se3(xi), gate)
            if len(source) < min_points:
                status = "insufficient_correspondences"
                break
            def residual(value):
                return np.sum((apply(camera @ exp_se3(value), source) - target) * normals, axis=1)
            fit = least_squares(residual, xi, loss="soft_l1", f_scale=.005, x_scale="jac", max_nfev=80,
                                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            step = np.linalg.norm(fit.x - xi)
            xi = fit.x
            history.append({"gate_mm": gate * 1000, "iteration": iteration, "correspondences": len(source),
                            "solver_success": bool(fit.success), "cost": float(fit.cost)})
            if not fit.success:
                status = "solver_not_converged"
            if step < 1e-7:
                converged = True
                break
        if not converged and status == "converged_schedule":
            status = "iteration_limit"
    correction = exp_se3(xi)
    corrected = camera @ correction
    source, target, normals = correspondences(usable, corrected, gates[-1])
    return corrected, {"status": status, "xi_rotation_deg": np.rad2deg(xi[:3]).tolist(),
                       "xi_translation_mm": (xi[3:] * 1000).tolist(),
                       "correction_translation_mm": (correction[:3, 3] * 1000).tolist(),
                       "translation_norm_mm": float(np.linalg.norm(correction[:3, 3]) * 1000),
                       "rotation_norm_deg": float(np.rad2deg(np.linalg.norm(xi[:3]))),
                       "corrected_T_base_color": corrected.tolist(), "iterations": history,
                       "observability": observability(source, target, normals, corrected)}


def cross_validate(groups, camera, gates, min_points):
    usable = [g for g in groups if np.count_nonzero(distances(g, camera)[0] <= gates[0]) >= min_points]
    result = {"unit": "pose_x_arm", "usable_groups": len(usable), "held_out": [],
              "status": "available" if len(usable) >= 2 else "fewer_than_2_usable_groups",
              "metric_note": "Same held-out points selected by the initial largest gate; no post-fit culling"}
    if len(usable) < 2:
        return result
    for index, group in enumerate(usable):
        fitted, fit = refine(usable[:index] + usable[index + 1:], camera, gates, min_points)
        before = distances(group, camera)[0]
        after = distances(group, fitted)[0]
        keep = before <= gates[0]
        result["held_out"].append({"pose": group.pose, "arm": group.arm, "point_count": int(keep.sum()),
                                   "fit_status": fit["status"], "fit_observability": fit["observability"],
                                   "before_mm": dict(zip(("median", "p95"), np.percentile(before[keep] * 1000, [50, 95]).tolist())),
                                   "after_mm": dict(zip(("median", "p95"), np.percentile(after[keep] * 1000, [50, 95]).tolist()))})
    return result


def overlay(snapshot, meshes, camera):
    image = snapshot["color"].copy()
    for arm, links in sorted(meshes.items()):
        mask = projected_mask(np.concatenate(list(links.values())), inverse(camera), snapshot["k"]["color"], image.shape[:2])
        selected = mask > 0
        image[selected] = np.rint(.65 * image[selected] + .35 * np.array(COLORS[arm])).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, COLORS[arm], 1)
    return image


def residual_image(snapshot, groups, camera, gates, pose_kind):
    image = np.zeros(snapshot["depth_m"].shape + (3,), np.uint8)
    best = np.full(snapshot["depth_m"].shape, np.inf)
    for group in groups:
        d = distances(group, camera)[0]
        y, x = group.pixels.T
        best[y, x] = np.minimum(best[y, x], d)
    visible = np.isfinite(best)
    image[visible] = (0, 0, 255)
    image[visible & (best <= .015)] = (0, 255, 255)
    image[visible & (best <= .005)] = (0, 200, 0)
    image[visible & (best > gates[0])] = (128, 128, 128)
    # Append a legend rather than painting over measured native-depth pixels.
    legend = np.full((64, max(560, image.shape[1]), 3), 25, np.uint8)
    lines = [f"{pose_kind}-pose residuals; ICP gates mm: " + ",".join(f"{g * 1000:g}" for g in gates),
             f"green <=5 mm; yellow <=15 mm; red >15 to {gates[0] * 1000:g} mm",
             f"grey >{gates[0] * 1000:g} mm (unmatched); black: no valid ROI depth"]
    for row, text in enumerate(lines):
        cv2.putText(legend, text, (6, 17 + row * 20), cv2.FONT_HERSHEY_SIMPLEX, .43, (230, 230, 230), 1, cv2.LINE_AA)
    padded = np.zeros((image.shape[0], legend.shape[1], 3), np.uint8)
    padded[:, :image.shape[1]] = image
    return np.concatenate((padded, legend))


def save_png(path, image):
    ok, encoded = cv2.imencode(".png", image)
    require(ok, f"Cannot encode {path}")
    with Path(path).open("xb") as stream:
        stream.write(encoded.tobytes())


def run_arm_check(args):
    require(args.confirm_static_same_pose, "arm-check requires --confirm-static-same-pose for every snapshot/state")
    output = Path(args.output)
    require(not output.exists() and not output.is_symlink(), f"Refusing existing output: {output}")
    require(type(args.roi_dilation_px) is int and 0 <= args.roi_dilation_px <= 2000, "ROI dilation must be in [0,2000] px")
    gates = [float(x) / 1000 for x in args.icp_gates_mm.split(",")]
    require(gates and all(positive(x) for x in gates) and all(a >= b for a, b in zip(gates, gates[1:])),
            "ICP gates must be finite positive decreasing millimetres")
    require(positive(args.surface_sample_spacing_mm) and positive(args.max_depth_m), "Spacing/max-depth must be finite positive")
    require(type(args.min_points_per_group) is int and args.min_points_per_group >= 6, "min-points-per-group must be >=6")
    links = args.links.split(",")
    require(all(links) and len(links) == len(set(links)), "--links must explicitly list unique nonempty link names")
    model = load_urdf(args.urdf, args.urdf_member)
    chains, owners = arm_chains(model, args.base, args.arm_chain, links)
    visuals = load_visuals(model, links, args.mesh_root)
    states = read_states(args.states)
    camera_record = read_json(args.camera_pose)
    require(isinstance(camera_record, dict) and camera_record.get("units") == "m", "Camera pose units must be m")
    require(camera_record.get("to_frame") == args.base, "Camera pose to_frame must match --base")
    require("matrix" in camera_record, "Camera pose missing matrix")
    camera = se3(camera_record["matrix"])
    snapshots, poses, groups = {}, {}, []
    for pose, directory in assignments(args.pose, "pose").items():
        snapshot = load_snapshot(directory)
        require(camera_record.get("from_frame") == snapshot["color_frame"], "Camera pose from_frame must match snapshot color frame_id")
        meshes = pose_meshes(model, visuals, args.base, chains, owners, states, pose)
        snapshots[pose], poses[pose] = snapshot, meshes
        groups.extend(make_groups(pose, snapshot, meshes, camera, args.roi_dilation_px,
                                  args.surface_sample_spacing_mm / 1000, args.max_depth_m))
    corrected, refinement = refine(groups, camera, gates, args.min_points_per_group)
    result = {
        "schema_version": 1, "status": "candidate_not_approved", "units": "m",
        "inputs": {"urdf": {"path": model.urdf_path, "member": model.urdf_member, "xml_sha256": model.urdf_sha256},
                   "mesh_root": str(Path(args.mesh_root).resolve()) if args.mesh_root else None,
                   "camera_pose": {"path": str(Path(args.camera_pose).resolve()), "sha256": sha256(args.camera_pose),
                                   "record": camera_record},
                   "states": {"path": str(Path(args.states).resolve()), "sha256": sha256(args.states)},
                   "snapshots": {pose: {"path": s["path"], "snapshot_json_sha256": s["sha256"]} for pose, s in snapshots.items()},
                   "base": args.base, "arm_chains": chains, "links": links, "confirm_static_same_pose": True},
        "parameters": {"roi_dilation_px": args.roi_dilation_px, "icp_gates_mm": [g * 1000 for g in gates],
                       "surface_sample_spacing_mm": args.surface_sample_spacing_mm, "max_depth_m": args.max_depth_m,
                       "min_points_per_group": args.min_points_per_group, "icp_loss": "soft_l1", "icp_f_scale_mm": 5,
                       "outer_iterations_per_gate": 15, "observability_weak_ratio": 1e-4,
                       "color_frame_selection": "first", "depth_aggregation": "median_nonzero_at_least_half_valid",
                       "residual_images": {"poses": ["input", "refined"], "green_max_mm": 5,
                                           "yellow_max_mm": 15, "grey_above_mm": gates[0] * 1000,
                                           "red": "above 15 mm through largest gate", "legend_appended": True},
                       "correction": "T_base_color @ exp(xi); xi=(rotation_rad, translation_tangent_m) in color optical"},
        "before": [group_stats(g, camera, gates, args.min_points_per_group) for g in groups],
        "refinement": refinement,
        "after": [group_stats(g, corrected, gates, args.min_points_per_group) for g in groups],
        "cross_validation": cross_validate(groups, camera, gates, args.min_points_per_group),
        "notes": ["D435 depth noise/bias can be a few millimetres; this is not metrology-grade calibration.",
                  "Vendor URDF and real arms differ (cables, covers, hands); only explicitly listed links are compared.",
                  "Confirmation flags do not prove synchronization or the same robot/camera pose.",
                  "Unsigned distances are to discrete surface samples, not exact triangles; spacing contributes to the distance floor.",
                  "ROIs are fixed from the input camera pose, with no occlusion removal; arms/background may overlap.",
                  "Normals follow STL winding; signed residual sign assumes consistent outward winding.",
                  "Weak observability, ICP local minima, depth bias and URDF mismatch can mimic an extrinsic correction.",
                  "Residual PNGs label input/refined poses: green <=5 mm, yellow <=15 mm, red >15 mm up to largest gate, grey unmatched beyond largest gate.",
                  "No acceptance decision is made; refinement is a diagnostic candidate only."],
    }
    output.mkdir(parents=True, exist_ok=False)
    for pose, snapshot in snapshots.items():
        save_png(output / f"pose_{pose}_overlay.png", overlay(snapshot, poses[pose], camera))
        save_png(output / f"pose_{pose}_overlay_refined.png", overlay(snapshot, poses[pose], corrected))
        pose_groups = [g for g in groups if g.pose == pose]
        save_png(output / f"pose_{pose}_depth_residual.png",
                 residual_image(snapshot, pose_groups, camera, gates, "input"))
        save_png(output / f"pose_{pose}_depth_residual_refined.png",
                 residual_image(snapshot, pose_groups, corrected, gates, "refined"))
    write_json(output / "arm_check.json", result)
    return result
