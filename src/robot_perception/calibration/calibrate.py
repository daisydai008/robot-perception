#!/usr/bin/env python3
"""First-stage calibration CLI; no hardware access except explicit capture --subscribe."""
import argparse
import json
from pathlib import Path
import sys

from .common import (CalibrationError, config_errors, load_config, make_board, positive,
                    provenance, read_json, require)


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    check = commands.add_parser("check-config", help="List target/protocol blockers without any hardware access")
    check.add_argument("--config", required=True)
    check.add_argument("--target", choices=("all", "board", "tags"), default="all")
    check.add_argument("--formal", action="store_true", help="Also require a frozen acceptance protocol")
    capture = commands.add_parser("capture", help="Finite camera-only subscription to an already running D435 driver")
    capture.add_argument("--config", required=True)
    capture.add_argument("--target", choices=("all", "board", "tags"), required=True)
    capture.add_argument("--output", required=True, help="New session directory (never overwritten)")
    capture.add_argument("--subscribe", action="store_true", help="Explicit on-site camera I/O authorization; does not authorize robot I/O")
    capture.add_argument("--samples", type=int, required=True, help="1–1000 complete RGB-D samples")
    capture.add_argument("--timeout-s", type=float, required=True, help="Finite total collection timeout, at most 3600 seconds")
    capture.add_argument("--interval-s", type=float, required=True, help="Positive spacing between saved samples")
    capture.add_argument("--max-pair-delta-ms", type=float, required=True, help="Maximum three-image header stamp span; no sync claim")
    capture.add_argument("--depth-unit-m", type=float, required=True, help="Confirmed metres per native/aligned uint16 unit; not assumed from encoding")
    capture.add_argument("--camera-serial", required=True, help="Operator-confirmed physical serial; not verified by this subscriber")
    capture.add_argument("--profile-record", required=True, help="Reference to operator-confirmed device/profile/driver parameter record")
    fit = commands.add_parser("intrinsics", help="Fit a raw-color Brown5 candidate and evaluate predeclared heldout images")
    fit.add_argument("--config", required=True)
    fit.add_argument("--manifest", required=True)
    fit.add_argument("--output", required=True, help="New candidate JSON file")
    tags = commands.add_parser("detect-tags", help="Offline AprilTag corners and overlay only, no pose or robot transform")
    tags.add_argument("--config", required=True)
    tags.add_argument("--image", required=True)
    tags.add_argument("--output", required=True, help="New result directory")
    reference = commands.add_parser("ee-reference", help="Offline URDF FK from recorded Tianji states; candidate T_base_ee only")
    reference.add_argument("--urdf", required=True, help="Vendor .urdf or .zip read in place")
    reference.add_argument("--urdf-member", help="Exact .urdf member in a zip; automatic only for a single URDF")
    reference.add_argument("--base", required=True)
    reference.add_argument("--ee", required=True)
    reference.add_argument("--arm", choices=("left", "right"), required=True)
    reference.add_argument("--samples", required=True, help="Text: one sample_id and pasted Tianji state line per image")
    reference.add_argument("--out-dir", required=True, help="One exclusive-create sample_id.json per state; never overwrite")
    reference.add_argument("--confirm-settled-same-pose", action="store_true",
                           help="Confirm EVERY state was read while settled at the same pose as its frozen image")
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "check-config":
            config = read_json(args.config)
            errors = config_errors(config, args.target, args.formal)
            if not errors and args.target in ("all", "board"):
                try:
                    make_board(config)
                except CalibrationError as exc:
                    errors.append(str(exc))
            print(json.dumps({"status": "blocked" if errors else "configuration_ready_not_calibrated",
                              "blockers": errors}, indent=2))
            return 2 if errors else 0
        if args.command == "capture":
            # Reject authorization/limits before opening config or importing capture/ROS.
            require(args.subscribe, "capture requires explicit --subscribe after on-site camera-only I/O approval")
            require(1 <= args.samples <= 1000, "samples must be in [1,1000]")
            require(positive(args.timeout_s) and args.timeout_s <= 3600, "timeout-s must be finite in (0,3600]")
            require(positive(args.interval_s), "interval-s must be finite and positive")
            require(positive(args.max_pair_delta_ms), "max-pair-delta-ms must be finite and positive")
            require(positive(args.depth_unit_m), "depth-unit-m must be confirmed, finite and positive")
            require(args.camera_serial.isascii() and args.camera_serial.isdigit(), "camera-serial must be the physical device's numeric serial")
            require(bool(args.profile_record.strip()), "profile-record must reference the operator-confirmed device/profile record")
            require(not Path(args.output).exists(), f"Refusing existing output: {args.output}")
            config = load_config(args.config, args.target, protocol=True)
            require(args.max_pair_delta_ms <= config["protocol"]["limits"]["rgb_depth_pair_delta_ms"],
                    "Requested pairing delta exceeds the frozen protocol")
            if args.target in ("all", "board"):
                make_board(config)
            from .capture import run_capture
            run_capture(args, provenance(args.config))
            print(f"Raw session saved to {args.output}; physical calibration remains unverified")
        elif args.command == "intrinsics":
            config = load_config(args.config, "board")
            from .offline import intrinsics
            result = intrinsics(args.config, config, args.manifest, args.output)
            print(json.dumps({key: result[key] for key in ("status", "train_summary", "heldout_summary")}, indent=2))
        elif args.command == "ee-reference":
            require(args.confirm_settled_same_pose, "ee-reference requires explicit --confirm-settled-same-pose")
            from .urdf_fk import write_ee_references
            count = write_ee_references(args)
            print(f"{count} candidate references saved to {args.out_dir}; vendor URDF/robot conventions remain unverified")
        elif args.command == "detect-tags":
            config = load_config(args.config, "tags")
            from .offline import detect_tags
            result = detect_tags(args.config, config, args.image, args.output)
            print(f"{len(result['detections'])} tags detected; detection only, not calibrated pose")
        return 0
    except (CalibrationError, OSError, ImportError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted; any partial capture is retained and is not a completed session", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
