#!/usr/bin/env python3
"""
Script to record reference poses for the SO100 robot arms.
Manually position the arms in good configurations and press Enter to record them.
These poses will be used as additional reference points for IK solving to prevent getting stuck.
"""

import os
import sys
import json
import time
import numpy as np
import logging
from pathlib import Path

# Add the telegrip package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from telegrip.config import TelegripConfig, REFERENCE_POSES_FILE, NUM_JOINTS
from telegrip.core.robot_interface import RobotInterface

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class PoseRecorder:
    """Records robot poses for use as IK reference poses."""

    def __init__(self):
        self.config = TelegripConfig()
        self.robot_interface = RobotInterface(self.config)
        self.reference_poses = {
            'left': [],
            'right': []
        }

    def connect_robot(self) -> bool:
        """Connect to the robot."""
        logger.info("Connecting to robot...")
        if not self.robot_interface.connect():
            logger.error("Failed to connect to robot")
            return False

        logger.info("✅ Robot connected successfully")
        return True

    def read_current_pose(self, arm: str) -> list:
        """Read current joint angles for specified arm from robot hardware."""
        try:
            angles = self.robot_interface.get_actual_arm_angles(arm)
            return angles.tolist()
        except Exception as e:
            logger.error(f"Failed to read current pose for {arm} arm: {e}")
            return None

    def record_arm_poses(self, arm: str):
        """Record poses for a single arm."""
        print(f"\n{'='*60}")
        print(f"Recording poses for {arm.upper()} ARM")
        print(f"{'='*60}")

        # Disable torque on this arm so it can be moved by hand
        print(f"\n🔓 Disabling torque on {arm.upper()} arm...")
        self.robot_interface.disable_torque(arm)
        print(f"✅ {arm.upper()} arm is now FREE - you can move it by hand\n")

        pose_count = 0

        while True:
            print(f"📍 {arm.upper()} ARM - Pose #{pose_count + 1}")
            print("   Position the arm in a good configuration...")

            user_input = input("   Press Enter to record, 's' to skip to next arm, 'done' to finish: ").strip().lower()

            if user_input == 'done':
                return 'done'
            elif user_input == 's':
                break

            # Read current pose
            current_pose = self.read_current_pose(arm)
            if current_pose is None:
                print("   ❌ Failed to read current pose. Try again.")
                continue

            # Add to reference poses
            self.reference_poses[arm].append(current_pose)
            pose_count += 1

            print(f"   ✅ Recorded pose #{pose_count}")
            print(f"   Angles: {[f'{x:.1f}°' for x in current_pose]}")

            # Show joint names for clarity
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
            for i, name in enumerate(joint_names):
                print(f"      {name}: {current_pose[i]:.1f}°")
            print()

            if pose_count >= 2:
                print(f"   ✅ Good! You have {pose_count} poses for {arm.upper()} arm.")
                print("   Record more, press 's' for next arm, or 'done' to finish.\n")

        return 'next'

    def record_poses(self):
        """Interactive pose recording session - one arm at a time."""
        print("\n" + "="*60)
        print("SO100 Robot Reference Pose Recorder")
        print("="*60)
        print("\nThis script will help you record reference poses for each arm.")
        print("These poses improve IK solving by providing good starting points.")
        print("\nInstructions:")
        print("  1. Each arm will be released one at a time")
        print("  2. Move the arm to different good configurations")
        print("  3. Press Enter to record each pose")
        print("  4. Press 's' to move to the next arm")
        print("  5. Type 'done' when completely finished")
        print("  6. Record 2-4 diverse poses per arm for best results")
        print("="*60)

        # Record left arm first
        result = self.record_arm_poses('left')
        if result == 'done':
            return

        # Then record right arm
        result = self.record_arm_poses('right')

    def save_poses(self):
        """Save recorded poses to file."""
        total_poses = len(self.reference_poses['left']) + len(self.reference_poses['right'])
        if total_poses == 0:
            print("❌ No poses recorded. Nothing to save.")
            return False

        # Get the configured file path (absolute)
        filename = Path(self.config.get_absolute_reference_poses_path())

        # Create parent directory if it doesn't exist
        filename.parent.mkdir(parents=True, exist_ok=True)

        try:
            with open(filename, 'w') as f:
                json.dump(self.reference_poses, f, indent=2)

            print(f"\n✅ Saved reference poses to {filename}")
            print(f"   Left arm:  {len(self.reference_poses['left'])} poses")
            print(f"   Right arm: {len(self.reference_poses['right'])} poses")
            print("\nThese poses will now be used by the IK solver.")
            return True

        except Exception as e:
            logger.error(f"Failed to save poses: {e}")
            return False

    def disconnect_robot(self):
        """Disconnect from the robot."""
        try:
            self.robot_interface.disconnect()
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")

def main():
    """Main function to read and save robot poses."""
    print("🤖 SO100 Reference Pose Recorder")
    print("=" * 40)

    recorder = PoseRecorder()

    try:
        # Connect to robot
        if not recorder.connect_robot():
            sys.exit(1)

        # Wait a moment for connection to stabilize
        time.sleep(0.5)

        # Record poses interactively
        recorder.record_poses()

        # Save recorded poses
        recorder.save_poses()

    except KeyboardInterrupt:
        print("\n\n🛑 Recording interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # Always disconnect
        recorder.disconnect_robot()

if __name__ == "__main__":
    main()
