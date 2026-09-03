#!/usr/bin/env python3
import argparse
import json
import math
import os
import time


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_NAME = "gripper"

JOINT_LIMITS = {
    "joint1": (-2.617994, 2.617994),
    "joint2": (0.0, 3.141593),
    "joint3": (-2.967060, 0.0),
    "joint4": (-1.745330, 1.745330),
    "joint5": (-1.221730, 1.221730),
    "joint6": (-2.094395, 2.094395),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a slower interpolated copy of a Piper recorded trajectory."
    )
    parser.add_argument(
        "--input",
        default="/home/user/piper_recorded_trajectory.json",
        help="Original recorded trajectory JSON path.",
    )
    parser.add_argument(
        "--output",
        default="/home/user/piper_smoothed_trajectory.json",
        help="Smoothed trajectory JSON path. The input file is never modified.",
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=0.05,
        help="Maximum allowed adjacent joint delta in rad after smoothing.",
    )
    parser.add_argument(
        "--min-dt",
        type=float,
        default=0.05,
        help="Minimum timestamp interval between inserted samples.",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.5,
        help="Slow down each original segment by this factor before adding safety spacing.",
    )
    parser.add_argument(
        "--limit-margin",
        type=float,
        default=0.20,
        help="Allow this much tiny feedback overshoot outside model limits, then clamp it back.",
    )
    return parser.parse_args()


def ensure_finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} is not finite: {value}")
    return value


def validate_joints(joints, limit_margin):
    if not isinstance(joints, list) or len(joints) != len(JOINT_NAMES):
        raise ValueError(f"Expected six joint values, got {joints!r}")

    values = [ensure_finite(value, name) for name, value in zip(JOINT_NAMES, joints)]
    for name, value in zip(JOINT_NAMES, values):
        low, high = JOINT_LIMITS[name]
        if value < low - limit_margin or value > high + limit_margin:
            raise ValueError(f"{name} value {value} is outside [{low}, {high}].")
    values = [
        max(low, min(high, value))
        for value, (low, high) in zip(values, (JOINT_LIMITS[name] for name in JOINT_NAMES))
    ]
    return values


def validate_sample(sample, index, limit_margin):
    if not isinstance(sample, dict):
        raise ValueError(f"Sample {index} is not an object.")
    joints = validate_joints(sample.get("joints"), limit_margin)
    t = ensure_finite(sample.get("t", 0.0), f"sample {index} timestamp")

    result = {"t": t, "joints": joints}
    if GRIPPER_NAME in sample:
        gripper = ensure_finite(sample[GRIPPER_NAME], f"sample {index} gripper")
        result[GRIPPER_NAME] = max(0.0, min(0.1, gripper))
    return result


def load_trajectory(path, limit_margin):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    samples = data.get("samples", [])
    if not samples:
        raise ValueError("Input trajectory contains no samples.")

    cleaned = [
        validate_sample(sample, i, limit_margin) for i, sample in enumerate(samples)
    ]
    for prev, cur in zip(cleaned, cleaned[1:]):
        if cur["t"] < prev["t"]:
            raise ValueError("Input timestamps must be nondecreasing.")

    data["joint_names"] = data.get("joint_names") or JOINT_NAMES
    return data, cleaned


def interpolate_list(start, end, ratio):
    return [a + (b - a) * ratio for a, b in zip(start, end)]


def interpolate_gripper(prev, cur, ratio):
    if GRIPPER_NAME not in prev and GRIPPER_NAME not in cur:
        return None
    start = prev.get(GRIPPER_NAME, cur.get(GRIPPER_NAME))
    end = cur.get(GRIPPER_NAME, start)
    width = start + (end - start) * ratio
    return max(0.0, min(0.1, width))


def smooth_samples(samples, max_step, min_dt, speed_scale):
    smoothed = [
        {
            "t": 0.0,
            "joints": samples[0]["joints"],
            **({GRIPPER_NAME: samples[0][GRIPPER_NAME]} if GRIPPER_NAME in samples[0] else {}),
        }
    ]

    output_t = 0.0
    max_original_step = 0.0
    inserted_points = 0

    for prev, cur in zip(samples, samples[1:]):
        joint_deltas = [abs(a - b) for a, b in zip(prev["joints"], cur["joints"])]
        max_delta = max(joint_deltas)
        max_original_step = max(max_original_step, max_delta)

        subdivisions = max(1, int(math.ceil(max_delta / max_step)))
        original_dt = max(0.0, cur["t"] - prev["t"])
        segment_dt = max(original_dt * speed_scale, subdivisions * min_dt)

        for step in range(1, subdivisions + 1):
            ratio = step / float(subdivisions)
            item = {
                "t": round(output_t + segment_dt * ratio, 4),
                "joints": interpolate_list(prev["joints"], cur["joints"], ratio),
            }
            gripper = interpolate_gripper(prev, cur, ratio)
            if gripper is not None:
                item[GRIPPER_NAME] = gripper
            smoothed.append(item)

        inserted_points += subdivisions - 1
        output_t += segment_dt

    return smoothed, max_original_step, inserted_points


def summarize(samples):
    if len(samples) < 2:
        return {
            "count": len(samples),
            "duration": 0.0,
            "max_step": 0.0,
            "min_dt": 0.0,
            "avg_dt": 0.0,
            "max_dt": 0.0,
        }

    max_step = 0.0
    for prev, cur in zip(samples, samples[1:]):
        max_step = max(
            max_step,
            max(abs(a - b) for a, b in zip(prev["joints"], cur["joints"])),
        )

    dts = [cur["t"] - prev["t"] for prev, cur in zip(samples, samples[1:])]
    return {
        "count": len(samples),
        "duration": samples[-1]["t"] - samples[0]["t"],
        "max_step": max_step,
        "min_dt": min(dts),
        "avg_dt": sum(dts) / len(dts),
        "max_dt": max(dts),
    }


def main():
    args = parse_args()
    if args.max_step <= 0:
        raise SystemExit("--max-step must be positive.")
    if args.min_dt <= 0:
        raise SystemExit("--min-dt must be positive.")
    if args.speed_scale < 1.0:
        raise SystemExit("--speed-scale must be >= 1.0.")

    if args.limit_margin < 0:
        raise SystemExit("--limit-margin must be >= 0.")

    source_data, samples = load_trajectory(args.input, args.limit_margin)
    smoothed, max_original_step, inserted_points = smooth_samples(
        samples, args.max_step, args.min_dt, args.speed_scale
    )

    output_data = {
        "format": "piper_joint_trajectory_v1",
        "source_file": args.input,
        "source_created_unix": source_data.get("created_unix"),
        "smoothed_created_unix": time.time(),
        "joint_names": JOINT_NAMES,
        "smoothing": {
            "max_step_rad": args.max_step,
            "min_dt_sec": args.min_dt,
            "speed_scale": args.speed_scale,
            "limit_margin_rad": args.limit_margin,
            "source_sample_count": len(samples),
            "inserted_points": inserted_points,
            "source_max_adjacent_joint_step_rad": max_original_step,
        },
        "samples": smoothed,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    tmp_path = args.output + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2)
        f.write("\n")
    os.replace(tmp_path, args.output)

    src = summarize(samples)
    dst = summarize(smoothed)
    print(f"input: {args.input}")
    print(f"output: {args.output}")
    print(
        "source: "
        f"{src['count']} samples, {src['duration']:.3f} s, "
        f"max step {src['max_step']:.4f} rad"
    )
    print(
        "smoothed: "
        f"{dst['count']} samples, {dst['duration']:.3f} s, "
        f"max step {dst['max_step']:.4f} rad, "
        f"dt min/avg/max {dst['min_dt']:.4f}/{dst['avg_dt']:.4f}/{dst['max_dt']:.4f} s"
    )


if __name__ == "__main__":
    main()
