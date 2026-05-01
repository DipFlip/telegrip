#!/usr/bin/env python3
"""
Visualize the reference poses stored in reference_poses.json using PyBullet.

Usage:
    python visualize_poses.py              # Manual mode (Enter to advance)
    python visualize_poses.py --auto       # Auto-cycle mode
    python visualize_poses.py --auto --delay 3.0
    python visualize_poses.py --poses-file my_poses.json
"""

import argparse
import json
import sys
import time
import os
import logging
import numpy as np
import pybullet as p

logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from telegrip.core.visualizer import PyBulletVisualizer
from telegrip.config import JOINT_NAMES, REFERENCE_POSES_FILE
from telegrip.utils import get_absolute_path


NEUTRAL_POSE = np.zeros(6)

# Per-arm label colors (RGB) for debug text
ARM_COLORS = {
    "left":  [0.2, 0.8, 1.0],
    "right": [1.0, 0.6, 0.2],
}


def load_poses(poses_file: str) -> dict:
    path = get_absolute_path(poses_file)
    if not path.exists():
        print(f"ERROR: Poses file not found: {path}")
        print("  Run read_pose.py first to record reference poses.")
        sys.exit(1)

    with open(path, "r") as f:
        data = json.load(f)

    for arm in ("left", "right"):
        count = len(data.get(arm, []))
        print(f"  {arm.upper()}: {count} pose(s)")

    return data


def build_pose_sequence(poses: dict) -> list[tuple[str, int, np.ndarray]]:
    """Return list of (arm, pose_index_1based, angles_deg)."""
    sequence = []
    for arm in ("left", "right"):
        for i, angles in enumerate(poses.get(arm, []), start=1):
            sequence.append((arm, i, np.array(angles, dtype=float)))
    return sequence


def clear_debug_text(text_ids: list) -> list:
    for tid in text_ids:
        try:
            p.removeUserDebugItem(tid)
        except Exception:
            pass
    return []


def show_pose_overlay(arm: str, pose_idx: int, total: int, angles: np.ndarray) -> list:
    """Draw joint-angle info in the PyBullet window. Returns list of text item IDs."""
    color = ARM_COLORS[arm]
    ids = []

    # Position overlay in upper-left area of the 3D scene
    base_x = -0.55
    base_y = 0.0
    base_z = 0.55
    line_step = 0.045

    header = f"[ {arm.upper()} ARM  |  Pose {pose_idx}/{total} ]"
    ids.append(p.addUserDebugText(
        header,
        [base_x, base_y, base_z],
        textColorRGB=color,
        textSize=1.4,
    ))

    for i, name in enumerate(JOINT_NAMES):
        label = f"  {name:<16} {angles[i]:+7.1f} deg"
        ids.append(p.addUserDebugText(
            label,
            [base_x, base_y, base_z - (i + 1) * line_step],
            textColorRGB=[0.9, 0.9, 0.9],
            textSize=1.1,
        ))

    hint = "Press Enter to continue..." if not _auto_mode else f"Next in {_auto_delay:.1f}s..."
    ids.append(p.addUserDebugText(
        hint,
        [base_x, base_y, base_z - (len(JOINT_NAMES) + 1.5) * line_step],
        textColorRGB=[0.5, 0.5, 0.5],
        textSize=1.0,
    ))

    return ids


# Module-level flags set by main() so show_pose_overlay can read them
_auto_mode = False
_auto_delay = 2.0


def run(viz: PyBulletVisualizer, sequence: list, auto: bool, delay: float):
    global _auto_mode, _auto_delay
    _auto_mode = auto
    _auto_delay = delay

    total = len(sequence)
    if total == 0:
        print("No poses found in file.")
        return

    print("\nControls:")
    if auto:
        print(f"  Auto-cycling every {delay}s  |  Ctrl+C to quit")
    else:
        print("  Press Enter to advance to the next pose  |  Ctrl+C to quit")

    text_ids: list = []
    pose_idx = 0

    try:
        while True:
            arm, idx_1based, angles = sequence[pose_idx % total]

            # Apply pose to the active arm, neutral to the other
            other_arm = "right" if arm == "left" else "left"
            viz.update_robot_pose(angles, arm)
            viz.update_robot_pose(NEUTRAL_POSE, other_arm)

            # Console output
            print(f"\n--- {arm.upper()} ARM  |  Pose {idx_1based}  ({pose_idx % total + 1}/{total}) ---")
            for i, name in enumerate(JOINT_NAMES):
                print(f"  {name:<16} {angles[i]:+7.1f}°")

            # PyBullet overlay
            text_ids = clear_debug_text(text_ids)
            text_ids = show_pose_overlay(arm, idx_1based, total, angles)

            if auto:
                # Keep window responsive while waiting
                deadline = time.time() + delay
                while time.time() < deadline:
                    viz.step_simulation()
                    time.sleep(0.05)
            else:
                input("\n  [Enter] next pose  |  Ctrl+C to quit > ")

            pose_idx += 1

    except KeyboardInterrupt:
        print("\nVisualization stopped.")
    finally:
        clear_debug_text(text_ids)


def main():
    parser = argparse.ArgumentParser(description="Visualize SO100 reference poses in PyBullet")
    parser.add_argument("--auto", action="store_true", help="Auto-cycle through poses")
    parser.add_argument("--delay", type=float, default=2.0, metavar="SECONDS",
                        help="Seconds between poses in auto mode (default: 2.0)")
    parser.add_argument("--poses-file", default=REFERENCE_POSES_FILE, metavar="FILE",
                        help=f"Path to poses JSON file (default: {REFERENCE_POSES_FILE})")
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", metavar="PATH",
                        help="Relative path to robot URDF (default: URDF/SO100/so100.urdf)")
    args = parser.parse_args()

    print("SO100/SO101 Reference Pose Visualizer")
    print("=" * 40)
    print(f"Poses file : {args.poses_file}")
    print(f"URDF       : {args.urdf}")

    poses = load_poses(args.poses_file)
    sequence = build_pose_sequence(poses)

    urdf_path = str(get_absolute_path(args.urdf))
    viz = PyBulletVisualizer(urdf_path=urdf_path, use_gui=True, log_level="warning")

    print("\nStarting PyBullet...")
    if not viz.setup():
        print("ERROR: Failed to initialize PyBullet.")
        sys.exit(1)

    print("PyBullet ready.\n")

    try:
        run(viz, sequence, auto=args.auto, delay=args.delay)
    finally:
        viz.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
