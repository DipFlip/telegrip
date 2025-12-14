"""
Robot interface module for the SO100 teleoperation system.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import numpy as np
import torch
import time
import logging
import os
import sys
import contextlib
from typing import Optional, Dict, Tuple

# New lerobot structure imports
from lerobot.robots.so100_follower.config_so100_follower import SO100FollowerConfig
from lerobot.robots.so100_follower.so100_follower import SO100Follower
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig

# Use our custom TelegripRobot (XLerobot without head motors)
from .telegrip_robot import TelegripRobot

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP,
    WHEEL_MOTOR_NAMES, BASE_SPEED_LEVELS, DEFAULT_BASE_SPEED_INDEX
)
from .kinematics import ForwardKinematics, IKSolver
from .omni_kinematics import body_to_wheel_velocities

# Joint name mapping: internal (telegrip) -> XLerobot format
XLEROBOT_LEFT_ARM_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}

XLEROBOT_RIGHT_ARM_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """High-level interface for SO100 robot control with safety features."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.robot_type = config.robot_type  # "so100" or "xlerobot"

        # For SO100 mode: separate robot instances
        self.left_robot = None
        self.right_robot = None

        # For XLerobot mode: single unified robot instance
        self.xlerobot = None

        self.is_connected = False
        self.is_engaged = False  # New state for motor engagement

        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # Joint state
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        
        # Joint limits (will be set by visualizer)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        # Kinematics solvers (will be set after PyBullet setup)
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}
        
        # Control timing
        self.last_send_time = 0
        
        # Error tracking - separate for each arm
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # Allow fewer errors per arm before marking as disconnected
        self.max_general_errors = 8  # Allow more general errors before full disconnection
        
        # Initial positions for safe shutdown - restored original values
        self.initial_left_arm = np.array([0, -100, 100, 60, 0, 0])
        self.initial_right_arm = np.array([0, -100, 100, 60, 0, 0])

        # Base (wheel) control state
        self.base_speed_index = DEFAULT_BASE_SPEED_INDEX
        self.current_base_velocities = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.wheel_commands = {name: 0 for name in WHEEL_MOTOR_NAMES}
    
    def setup_robot_configs(self) -> Tuple[SO100FollowerConfig, SO100FollowerConfig]:
        """Create robot configurations for both arms."""
        logger.info(f"Setting up robot configs with ports: {self.config.follower_ports}")
        
        left_config = SO100FollowerConfig(
            port=self.config.follower_ports["left"],
            use_degrees=True,  # Use degrees for easier debugging
            disable_torque_on_disconnect=True
        )
        # Set the robot name for calibration file lookup
        left_config.id = "left_follower"
        
        right_config = SO100FollowerConfig(
            port=self.config.follower_ports["right"],
            use_degrees=True,  # Use degrees for easier debugging
            disable_torque_on_disconnect=True
        )
        # Set the robot name for calibration file lookup
        right_config.id = "right_follower"
        
        return left_config, right_config
    
    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True  # Mark as "connected" for testing
            return True

        # Setup suppression if requested
        should_suppress = (self.config.log_level == "warning" or
                          self.config.log_level == "critical" or
                          self.config.log_level == "error")

        try:
            if not should_suppress:
                logger.info(f"Connecting to robot (type: {self.robot_type})...")

            if self.robot_type == "xlerobot":
                return self._connect_xlerobot(should_suppress)
            else:
                return self._connect_so100(should_suppress)

        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    def _connect_xlerobot(self, should_suppress: bool) -> bool:
        """Connect using XLerobot (unified dual-arm + wheels)."""
        try:
            xlerobot_config = XLerobotConfig(
                port1=self.config.follower_ports["left"],  # Left arm + head
                port2=self.config.follower_ports["right"],  # Right arm + wheels
                use_degrees=True,
                disable_torque_on_disconnect=True
            )
            xlerobot_config.id = "xlerobot"

            if should_suppress:
                with suppress_stdout_stderr():
                    self.xlerobot = TelegripRobot(xlerobot_config)
                    self.xlerobot.connect()
            else:
                self.xlerobot = TelegripRobot(xlerobot_config)
                self.xlerobot.connect()

            # Both arms connected via single XLerobot instance
            self.left_arm_connected = True
            self.right_arm_connected = True
            self.is_connected = True

            # Initialize joint states
            self._read_initial_state()
            logger.info("✅ XLerobot connected (left arm + right arm + wheels)")
            return True

        except Exception as e:
            logger.error(f"❌ XLerobot connection failed: {e}")
            self.is_connected = False
            return False

    def _connect_so100(self, should_suppress: bool) -> bool:
        """Connect using two separate SO100Follower instances."""
        left_config, right_config = self.setup_robot_configs()

        # Connect left arm
        try:
            if should_suppress:
                with suppress_stdout_stderr():
                    self.left_robot = SO100Follower(left_config)
                    self.left_robot.connect()
            else:
                self.left_robot = SO100Follower(left_config)
                self.left_robot.connect()
            self.left_arm_connected = True
            logger.info("✅ Left arm connected successfully")
        except Exception as e:
            logger.error(f"❌ Left arm connection failed: {e}")
            self.left_arm_connected = False

        # Connect right arm
        try:
            if should_suppress:
                with suppress_stdout_stderr():
                    self.right_robot = SO100Follower(right_config)
                    self.right_robot.connect()
            else:
                self.right_robot = SO100Follower(right_config)
                self.right_robot.connect()
            self.right_arm_connected = True
            logger.info("✅ Right arm connected successfully")
        except Exception as e:
            logger.error(f"❌ Right arm connection failed: {e}")
            self.right_arm_connected = False

        # Mark as connected if at least one arm is connected
        self.is_connected = self.left_arm_connected or self.right_arm_connected

        if self.is_connected:
            # Initialize joint states
            self._read_initial_state()
            logger.info(f"🤖 Robot interface connected: Left={self.left_arm_connected}, Right={self.right_arm_connected}")
        else:
            logger.error("❌ Failed to connect any robot arms")

        return self.is_connected
    
    def _read_initial_state(self):
        """Read initial joint state from robot."""
        try:
            if self.robot_type == "xlerobot" and self.xlerobot:
                observation = self.xlerobot.get_observation()
                if observation:
                    # XLerobot uses prefixed joint names
                    self.left_arm_angles = np.array([
                        observation['left_arm_shoulder_pan.pos'],
                        observation['left_arm_shoulder_lift.pos'],
                        observation['left_arm_elbow_flex.pos'],
                        observation['left_arm_wrist_flex.pos'],
                        observation['left_arm_wrist_roll.pos'],
                        observation['left_arm_gripper.pos']
                    ])
                    self.right_arm_angles = np.array([
                        observation['right_arm_shoulder_pan.pos'],
                        observation['right_arm_shoulder_lift.pos'],
                        observation['right_arm_elbow_flex.pos'],
                        observation['right_arm_wrist_flex.pos'],
                        observation['right_arm_wrist_roll.pos'],
                        observation['right_arm_gripper.pos']
                    ])
                    logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")
                    logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")
            else:
                # SO100 mode: separate robots
                if self.left_robot and self.left_arm_connected:
                    observation = self.left_robot.get_observation()
                    if observation:
                        self.left_arm_angles = np.array([
                            observation['shoulder_pan.pos'],
                            observation['shoulder_lift.pos'],
                            observation['elbow_flex.pos'],
                            observation['wrist_flex.pos'],
                            observation['wrist_roll.pos'],
                            observation['gripper.pos']
                        ])
                        logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")

                if self.right_robot and self.right_arm_connected:
                    observation = self.right_robot.get_observation()
                    if observation:
                        self.right_arm_angles = np.array([
                            observation['shoulder_pan.pos'],
                            observation['shoulder_lift.pos'],
                            observation['elbow_flex.pos'],
                            observation['wrist_flex.pos'],
                            observation['wrist_roll.pos'],
                            observation['gripper.pos']
                        ])
                        logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")

        except Exception as e:
            logger.error(f"Error reading initial state: {e}")
    
    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict, 
                        end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray, 
                        joint_limits_max_deg: np.ndarray):
        """Setup kinematics solvers using PyBullet components for both arms."""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()
        
        # Setup solvers for both arms
        for arm in ['left', 'right']:
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
            )
            
            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
                joint_limits_min_deg, joint_limits_max_deg, arm_name=arm
            )
        
        logger.info("Kinematics solvers initialized for both arms")
    
    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """Get current end effector position for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        if self.fk_solvers[arm]:
            position, _ = self.fk_solvers[arm].compute(angles)
            return position
        else:
            default_position = np.array([0.2, 0.0, 0.15])
            return default_position
    
    def solve_ik(self, arm: str, target_position: np.ndarray, 
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve inverse kinematics for specified arm."""
        if arm == "left":
            current_angles = self.left_arm_angles
        elif arm == "right":
            current_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(target_position, target_orientation, current_angles)
        else:
            return current_angles[:3]  # Return current angles if no IK solver
    
    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """Clamp joint angles to safe limits with margins for problem joints."""
        # Create a copy to avoid modifying the original
        processed_angles = joint_angles.copy()
        
        # First, normalize angles that can wrap around (like shoulder_pan)
        # Check if first joint (shoulder_pan) is outside limits but can be wrapped
        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = self.joint_limits_min_deg[shoulder_pan_idx]  # -120.3°
        max_limit = self.joint_limits_max_deg[shoulder_pan_idx]  # +120.3°
        
        # Try to wrap the angle to an equivalent angle within limits
        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            # Try wrapping by ±360°
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    logger.debug(f"Wrapped shoulder_pan from {shoulder_pan_angle:.1f}° to {wrapped_angle:.1f}°")
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break
        
        # Apply standard joint limits to all joints
        return np.clip(processed_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)
    
    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float):
        """Update joint angles for specified arm with IK solution and direct wrist/gripper control."""
        if arm == "left":
            target_angles = self.left_arm_angles
        elif arm == "right":
            target_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        # Update first 3 joints with IK solution
        target_angles[:3] = ik_angles
        
        # Set wrist angles directly
        target_angles[3] = wrist_flex
        target_angles[4] = wrist_roll
        
        # Handle gripper separately (clamp to gripper limits)
        target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        
        # Apply joint limits to all joints (except gripper which we handle specially)
        clamped_angles = self.clamp_joint_angles(target_angles)
        
        # Preserve gripper control (don't clamp gripper if it was set intentionally)
        clamped_angles[5] = target_angles[5]
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles
    
    def engage(self) -> bool:
        """Engage robot motors (start sending commands)."""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False
        
        self.is_engaged = True
        logger.info("🔌 Robot motors ENGAGED - commands will be sent")
        return True
    
    def disengage(self) -> bool:
        """Disengage robot motors (stop sending commands)."""
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True
        
        try:
            # Return to safe position before disengaging
            self.return_to_initial_position()
            
            # Disable torque
            self.disable_torque()
            
            self.is_engaged = False
            logger.info("🔌 Robot motors DISENGAGED - commands stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error disengaging robot: {e}")
            return False
    
    def send_command(self) -> bool:
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_engaged:
            return False

        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently

        try:
            if self.robot_type == "xlerobot":
                return self._send_command_xlerobot(current_time)
            else:
                return self._send_command_so100(current_time)

        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ Robot interface disconnected due to repeated errors")
            return False

    def _send_command_xlerobot(self, current_time: float) -> bool:
        """Send command using XLerobot (unified action dict with prefixed names)."""
        if not self.xlerobot:
            return False

        try:
            # Debug: Log wrist angles being sent
            logger.debug(f"🔧 Sending wrist angles - Left flex/roll: {self.left_arm_angles[3]:.1f}/{self.left_arm_angles[4]:.1f}, Right flex/roll: {self.right_arm_angles[3]:.1f}/{self.right_arm_angles[4]:.1f}")

            # Build unified action dict with all joints + wheel velocities
            # XLerobot uses x.vel, y.vel, theta.vel and does kinematics internally
            action_dict = {
                # Left arm (prefixed names)
                "left_arm_shoulder_pan.pos": float(self.left_arm_angles[0]),
                "left_arm_shoulder_lift.pos": float(self.left_arm_angles[1]),
                "left_arm_elbow_flex.pos": float(self.left_arm_angles[2]),
                "left_arm_wrist_flex.pos": float(self.left_arm_angles[3]),
                "left_arm_wrist_roll.pos": float(self.left_arm_angles[4]),
                "left_arm_gripper.pos": float(self.left_arm_angles[5]),
                # Right arm (prefixed names)
                "right_arm_shoulder_pan.pos": float(self.right_arm_angles[0]),
                "right_arm_shoulder_lift.pos": float(self.right_arm_angles[1]),
                "right_arm_elbow_flex.pos": float(self.right_arm_angles[2]),
                "right_arm_wrist_flex.pos": float(self.right_arm_angles[3]),
                "right_arm_wrist_roll.pos": float(self.right_arm_angles[4]),
                "right_arm_gripper.pos": float(self.right_arm_angles[5]),
                # Base velocities (XLerobot handles wheel kinematics internally)
                "x.vel": float(self.current_base_velocities["x"]),
                "y.vel": float(self.current_base_velocities["y"]),
                "theta.vel": float(self.current_base_velocities["theta"]),
            }

            # Debug: log when base velocities are non-zero
            if (self.current_base_velocities["x"] != 0 or
                self.current_base_velocities["y"] != 0 or
                self.current_base_velocities["theta"] != 0):
                logger.debug(f"🛞 Sending base velocities: x={self.current_base_velocities['x']:.3f}, y={self.current_base_velocities['y']:.3f}, theta={self.current_base_velocities['theta']:.1f}")

            self.xlerobot.send_action(action_dict)
            self.last_send_time = current_time
            return True

        except Exception as e:
            logger.error(f"Error sending XLerobot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ XLerobot disconnected due to repeated errors")
            return False

    def _send_command_so100(self, current_time: float) -> bool:
        """Send command using two separate SO100Follower instances."""
        success = True

        # Send left arm command
        if self.left_robot and self.left_arm_connected:
            try:
                action_dict = {
                    "shoulder_pan.pos": float(self.left_arm_angles[0]),
                    "shoulder_lift.pos": float(self.left_arm_angles[1]),
                    "elbow_flex.pos": float(self.left_arm_angles[2]),
                    "wrist_flex.pos": float(self.left_arm_angles[3]),
                    "wrist_roll.pos": float(self.left_arm_angles[4]),
                    "gripper.pos": float(self.left_arm_angles[5])
                }
                self.left_robot.send_action(action_dict)
            except Exception as e:
                logger.error(f"Error sending left arm command: {e}")
                self.left_arm_errors += 1
                if self.left_arm_errors > self.max_arm_errors:
                    self.left_arm_connected = False
                    logger.error("❌ Left arm disconnected due to repeated errors")
                success = False

        # Send right arm command (SO100 doesn't support wheels)
        if self.right_robot and self.right_arm_connected:
            try:
                action_dict = {
                    "shoulder_pan.pos": float(self.right_arm_angles[0]),
                    "shoulder_lift.pos": float(self.right_arm_angles[1]),
                    "elbow_flex.pos": float(self.right_arm_angles[2]),
                    "wrist_flex.pos": float(self.right_arm_angles[3]),
                    "wrist_roll.pos": float(self.right_arm_angles[4]),
                    "gripper.pos": float(self.right_arm_angles[5]),
                }
                self.right_robot.send_action(action_dict)
            except Exception as e:
                logger.error(f"Error sending right arm command: {e}")
                self.right_arm_errors += 1
                if self.right_arm_errors > self.max_arm_errors:
                    self.right_arm_connected = False
                    logger.error("❌ Right arm disconnected due to repeated errors")
                success = False

        self.last_send_time = current_time
        return success
    
    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE

        if arm == "left":
            self.left_arm_angles[5] = angle
        elif arm == "right":
            self.right_arm_angles[5] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")

    # --- Base (Wheel) Control Methods ---

    def set_base_velocities(self, x_vel: float, y_vel: float, theta_vel: float):
        """Set base velocities.

        Args:
            x_vel: Forward/backward velocity (m/s), positive = forward
            y_vel: Strafe left/right velocity (m/s), positive = left
            theta_vel: Rotational velocity (deg/s), positive = counter-clockwise

        For XLerobot: velocities are sent directly as x.vel, y.vel, theta.vel
        For SO100: not supported (no wheels)
        """
        self.current_base_velocities = {"x": x_vel, "y": y_vel, "theta": theta_vel}
        # For SO100 compatibility (not used but kept for interface consistency)
        self.wheel_commands = body_to_wheel_velocities(x_vel, y_vel, theta_vel)

    def stop_base(self):
        """Stop all wheel motors (set velocities to zero)."""
        self.set_base_velocities(0.0, 0.0, 0.0)

    def get_current_speed_level(self) -> dict:
        """Get current speed level settings."""
        return BASE_SPEED_LEVELS[self.base_speed_index]

    def get_max_linear_speed(self) -> float:
        """Get current maximum linear speed (m/s)."""
        return BASE_SPEED_LEVELS[self.base_speed_index]["linear"]

    def get_max_angular_speed(self) -> float:
        """Get current maximum angular speed (deg/s)."""
        return BASE_SPEED_LEVELS[self.base_speed_index]["angular"]

    def increase_base_speed(self) -> int:
        """Increase base speed level. Returns new speed index."""
        if self.base_speed_index < len(BASE_SPEED_LEVELS) - 1:
            self.base_speed_index += 1
            logger.info(f"Base speed increased to level {self.base_speed_index + 1}: {self.get_current_speed_level()}")
        return self.base_speed_index

    def decrease_base_speed(self) -> int:
        """Decrease base speed level. Returns new speed index."""
        if self.base_speed_index > 0:
            self.base_speed_index -= 1
            logger.info(f"Base speed decreased to level {self.base_speed_index + 1}: {self.get_current_speed_level()}")
        return self.base_speed_index
    
    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        return angles
    
    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm, for PyBullet visualization."""
        # Return raw angles without any correction for proper diagnosis
        return self.get_arm_angles(arm)
    
    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """Get actual joint angles from robot hardware (not commanded angles)."""
        try:
            if self.robot_type == "xlerobot" and self.xlerobot:
                observation = self.xlerobot.get_observation()
                if observation:
                    if arm == "left":
                        return np.array([
                            observation['left_arm_shoulder_pan.pos'],
                            observation['left_arm_shoulder_lift.pos'],
                            observation['left_arm_elbow_flex.pos'],
                            observation['left_arm_wrist_flex.pos'],
                            observation['left_arm_wrist_roll.pos'],
                            observation['left_arm_gripper.pos']
                        ])
                    elif arm == "right":
                        return np.array([
                            observation['right_arm_shoulder_pan.pos'],
                            observation['right_arm_shoulder_lift.pos'],
                            observation['right_arm_elbow_flex.pos'],
                            observation['right_arm_wrist_flex.pos'],
                            observation['right_arm_wrist_roll.pos'],
                            observation['right_arm_gripper.pos']
                        ])
            else:
                # SO100 mode
                if arm == "left" and self.left_robot and self.left_arm_connected:
                    observation = self.left_robot.get_observation()
                    if observation:
                        return np.array([
                            observation['shoulder_pan.pos'],
                            observation['shoulder_lift.pos'],
                            observation['elbow_flex.pos'],
                            observation['wrist_flex.pos'],
                            observation['wrist_roll.pos'],
                            observation['gripper.pos']
                        ])
                elif arm == "right" and self.right_robot and self.right_arm_connected:
                    observation = self.right_robot.get_observation()
                    if observation:
                        return np.array([
                            observation['shoulder_pan.pos'],
                            observation['shoulder_lift.pos'],
                            observation['elbow_flex.pos'],
                            observation['wrist_flex.pos'],
                            observation['wrist_roll.pos'],
                            observation['gripper.pos']
                        ])
        except Exception as e:
            logger.debug(f"Error reading actual arm angles for {arm}: {e}")

        # Fallback to commanded angles if we can't read actual angles
        return self.get_arm_angles(arm)
    
    def return_to_initial_position(self):
        """Return both arms to initial position."""
        logger.info("⏪ Returning robot to initial position...")
        
        try:
            # Set initial positions - no direction mapping
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()
            
            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()
                time.sleep(0.1)
                
            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")
    
    def disable_torque(self, arm: str = None):
        """Disable torque on robot joints.

        Args:
            arm: 'left', 'right', or None for both arms
        """
        if not self.is_connected:
            return

        try:
            if self.robot_type == "xlerobot" and self.xlerobot:
                # XLerobot has two buses
                if arm is None or arm == "left":
                    logger.info("Disabling torque on LEFT arm (bus1)...")
                    self.xlerobot.bus1.disable_torque()
                if arm is None or arm == "right":
                    logger.info("Disabling torque on RIGHT arm + wheels (bus2)...")
                    self.xlerobot.bus2.disable_torque()
            else:
                # SO100 mode
                if arm is None or arm == "left":
                    if self.left_robot and self.left_arm_connected:
                        logger.info("Disabling torque on LEFT arm...")
                        self.left_robot.bus.disable_torque()

                if arm is None or arm == "right":
                    if self.right_robot and self.right_arm_connected:
                        logger.info("Disabling torque on RIGHT arm...")
                        self.right_robot.bus.disable_torque()

        except Exception as e:
            logger.error(f"Error disabling torque: {e}")
    
    def disconnect(self):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return

        logger.info("Disconnecting from robot...")

        # Return to initial positions if engaged
        if self.is_engaged:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"Error returning to initial position: {e}")

        if self.robot_type == "xlerobot" and self.xlerobot:
            # XLerobot: single disconnect call
            try:
                self.xlerobot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting XLerobot: {e}")
            self.xlerobot = None
        else:
            # SO100 mode: disconnect both arms separately
            if self.left_robot:
                try:
                    self.left_robot.disconnect()
                except Exception as e:
                    logger.error(f"Error disconnecting left arm: {e}")
                self.left_robot = None

            if self.right_robot:
                try:
                    self.right_robot.disconnect()
                except Exception as e:
                    logger.error(f"Error disconnecting right arm: {e}")
                self.right_robot = None

        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 Robot disconnected")
    
    def get_arm_connection_status(self, arm: str) -> bool:
        """Get connection status for specific arm based on device file existence."""
        # Only check device file existence - ignore overall robot connection status
        if arm == "left":
            device_path = self.config.follower_ports["left"]
            return os.path.exists(device_path)
        elif arm == "right":
            device_path = self.config.follower_ports["right"] 
            return os.path.exists(device_path)
        else:
            return False

    def update_arm_connection_status(self):
        """Update individual arm connection status based on device file existence."""
        if self.is_connected:
            self.left_arm_connected = os.path.exists(self.config.follower_ports["left"])
            self.right_arm_connected = os.path.exists(self.config.follower_ports["right"])
    
    @property
    def status(self) -> Dict:
        """Get robot status information."""
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": self.joint_limits_min_deg.tolist(),
            "joint_limits_max": self.joint_limits_max_deg.tolist(),
            # Base control status
            "base_speed_index": self.base_speed_index,
            "base_speed_level": self.get_current_speed_level(),
            "base_velocities": self.current_base_velocities,
        } 