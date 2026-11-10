"""Offline image measurements. Results are candidates, never deployment approvals."""
from pathlib import Path

from .common import (board_points, detect_board, detect_markers, make_board, provenance,
                    read_json, require, sha256, vision, write_json)


def load_image(path):
    cv2, _ = vision()
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    require(image is not None, f"Cannot read image: {path}")
    return image


def residual(obj, img, rvec, tvec, k, d):
    cv2, np = vision()
    projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
    lengths = np.linalg.norm(projected.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
    return {"corners": len(lengths), "rms_px": float(np.sqrt(np.mean(lengths ** 2))),
            "max_px": float(lengths.max())}


def check_diversity(objects, images):
    """Zhang planar constraints must constrain all five intrinsic DOFs.

    This is a numerical degeneracy guard, not an approved physical accuracy limit.
    """
    cv2, np = vision()
    rows = []
    for obj, img in zip(objects, images):
        h, _ = cv2.findHomography(obj[:, :2], img.reshape(-1, 2), method=0)
        require(h is not None and np.isfinite(h).all(), "Degenerate board homography")
        # Normalize pixel coordinates to improve conditioning without modifying images.
        h = np.diag([1 / 1000, 1 / 1000, 1]) @ h

        def v(i, j):
            return np.array([h[0, i] * h[0, j], h[0, i] * h[1, j] + h[1, i] * h[0, j],
                             h[1, i] * h[1, j], h[2, i] * h[0, j] + h[0, i] * h[2, j],
                             h[2, i] * h[1, j] + h[1, i] * h[2, j], h[2, i] * h[2, j]])
        for row in (v(0, 1), v(0, 0) - v(1, 1)):
            require(np.linalg.norm(row) > 0, "Degenerate planar constraints")
            rows.append(row / np.linalg.norm(row))
    values = np.linalg.svd(rows, compute_uv=False)
    require(values[4] / values[0] > 1e-4,
            "Insufficient view diversity: acquire varied distances, image positions and multi-axis tilts")
    return values.tolist()


def fit_intrinsics(objects, images, size):
    cv2, np = vision()
    require(len(objects) >= 6, "Need at least 6 usable training views (solver minimum, not a formal sample budget)")
    spectrum = check_diversity(objects, images)
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(objects, images, size, None, None)
    require(all(np.isfinite(x).all() for x in [k, d, np.array(rvecs), np.array(tvecs)]) and np.isfinite(rms),
            "Calibration produced non-finite parameters")
    require(k[0, 0] > 0 and k[1, 1] > 0 and 0 < k[0, 2] < size[0] and 0 < k[1, 2] < size[1],
            "Implausible camera matrix; inspect board layout, coverage and pixel geometry")
    metrics = []
    for obj, img, rvec, tvec in zip(objects, images, rvecs, tvecs):
        rotation, _ = cv2.Rodrigues(rvec)
        require(np.all((obj @ rotation.T + tvec.reshape(1, 3))[:, 2] > 0), "Training pose behind camera")
        metrics.append(residual(obj, img, rvec, tvec, k, d))
    return k, d, {"opencv_training_rms_px": float(rms), "constraint_singular_values": spectrum}, metrics


def evaluate_pose(obj, img, k, d):
    cv2, np = vision()
    ok, rvec, tvec = cv2.solvePnP(obj, img, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        return {"status": "pose_fit_failed"}
    rotation, _ = cv2.Rodrigues(rvec)
    if not np.all((obj @ rotation.T + tvec.reshape(1, 3))[:, 2] > 0):
        return {"status": "pose_behind_camera"}
    return {"status": "ok", **residual(obj, img, rvec, tvec, k, d)}


def intrinsics(config_path, config, manifest_path, output):
    cv2, np = vision()
    require(not Path(output).exists(), f"Refusing existing output: {output}")
    manifest_path = Path(manifest_path)
    manifest = read_json(manifest_path)
    require(isinstance(manifest, dict), "Manifest must be an object")
    require(manifest.get("image_geometry") == "raw_color_no_crop_no_resize", "Manifest must declare raw_color_no_crop_no_resize")
    require(manifest.get("candidate_model") == "opencv_brown5", "Only candidate opencv_brown5 fitting is implemented")
    require(isinstance(manifest.get("split_frozen_on"), str) and bool(manifest["split_frozen_on"].strip()),
            "Declare train/heldout split_frozen_on before solving")
    require(isinstance(manifest.get("camera_serial"), str) and bool(manifest["camera_serial"].strip()), "Manifest requires camera_serial")
    size = manifest.get("image_size")
    require(isinstance(size, list) and len(size) == 2 and all(type(x) is int and x > 0 for x in size),
            "Manifest image_size must be [width,height] of original pixels")
    paths = {}
    hashes = set()
    for split, minimum in (("train", 6), ("heldout", 3)):
        names = manifest.get(split)
        require(isinstance(names, list) and len(names) >= minimum and all(isinstance(n, str) for n in names),
                f"Manifest requires at least {minimum} explicit {split} image paths")
        paths[split] = []
        for name in names:
            path = (manifest_path.parent / name).resolve()
            digest = sha256(path)
            require(digest not in hashes, f"Duplicate image content across/within splits: {name}")
            hashes.add(digest)
            paths[split].append((path, digest))
    board, dictionary = make_board(config)
    points = board_points(board)
    reports, usable = {}, {}
    for split in paths:
        reports[split], usable[split] = [], []
        for path, digest in paths[split]:
            image = load_image(path)
            require((image.shape[1], image.shape[0]) == tuple(size), f"Image size changed/cropped: {path}")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_board(gray, board, dictionary)
            report = {"image": str(path), "sha256": digest, "detected_corners": 0 if ids is None else len(ids)}
            reports[split].append(report)
            if ids is None or len(ids) < 6:
                report["status"] = "insufficient_corners"
                continue
            obj = points[ids.reshape(-1)].astype(np.float32)
            if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(axis=0)) < 2:
                report["status"] = "collinear_corners"
                continue
            report["status"] = "detected"
            usable[split].append((obj, corners.astype(np.float32), report))
    result = {"schema_version": 1, "status": "not_solved", "target_config": provenance(config_path),
              "manifest": provenance(manifest_path), "opencv_version": cv2.__version__, "image_size": size,
              "candidate_model": "opencv_brown5", "views": reports,
              "interpretation": "Heldout K,D stay fixed but each board pose is fitted; residuals are not independent absolute accuracy. No acceptance or driver update."}
    if len(usable["train"]) < 6 or len(usable["heldout"]) < 3:
        write_json(output, result)
        require(False, f"Too few usable views: train={len(usable['train'])}, heldout={len(usable['heldout'])}; detection failures saved to {output}")
    from .common import CalibrationError
    try:
        k, d, summary, train_metrics = fit_intrinsics([x[0] for x in usable["train"]],
                                                     [x[1] for x in usable["train"]], tuple(size))
    except (CalibrationError, cv2.error) as exc:
        result["solve_error"] = str(exc)
        write_json(output, result)
        raise CalibrationError(f"Intrinsics not solved; diagnostic saved to {output}: {exc}") from exc
    for (_, _, report), metrics in zip(usable["train"], train_metrics):
        report.update(status="ok", **metrics)
    for obj, img, report in usable["heldout"]:
        try:
            report.update(evaluate_pose(obj, img, k, d))
        except cv2.error as exc:
            report.update(status="pose_fit_failed", error=str(exc))
    result.update(status="candidate_not_approved", k=k.tolist(), d=d.reshape(-1).tolist(), fit=summary)
    for split in reports:
        good = [r for r in reports[split] if r["status"] == "ok"]
        pooled_rms = None
        if good:
            pooled_rms = float(np.sqrt(sum(r["rms_px"] ** 2 * r["corners"] for r in good)
                                       / sum(r["corners"] for r in good)))
        result[split + "_summary"] = {"total": len(reports[split]), "successful": len(good),
                                      "failed": len(reports[split]) - len(good),
                                      "pooled_corner_rms_px": pooled_rms}
    write_json(output, result)
    require(all(r["status"] == "ok" for r in reports["heldout"]),
            f"Heldout failures recorded in {output}; candidate is not validated")
    return result


def detect_tags(config_path, config, image_path, output):
    cv2, _ = vision()
    output = Path(output)
    require(not output.exists(), f"Refusing existing output: {output}")
    image = load_image(image_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners, ids, _ = detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), dictionary)
    records = []
    sizes = config["apriltags"]["side_lengths_m"]
    if ids is not None:
        for corner, marker_id in zip(corners, ids.reshape(-1)):
            key = str(int(marker_id))
            records.append({"id": int(marker_id), "corners_px": corner.reshape(4, 2).tolist(),
                            "selected": key in sizes, "measured_black_border_m": sizes.get(key)})
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
    ok, png = cv2.imencode(".png", image)
    require(ok, "Could not encode detection overlay")
    output.mkdir(parents=True, exist_ok=False)
    with (output / "overlay.png").open("xb") as stream:
        stream.write(png.tobytes())
    result = {"status": "detection_only_not_pose", "target_config": provenance(config_path),
              "image": str(Path(image_path).resolve()), "image_sha256": sha256(image_path),
              "image_size": [image.shape[1], image.shape[0]], "opencv_version": cv2.__version__,
              "dictionary": "DICT_APRILTAG_36h11", "detections": records,
              "missing_selected_ids": sorted(int(key) for key in sizes if int(key) not in [r["id"] for r in records]),
              "corner_order": "OpenCV marker order: top-left, top-right, bottom-right, bottom-left in decoded marker orientation"}
    write_json(output / "detections.json", result)
    return result
