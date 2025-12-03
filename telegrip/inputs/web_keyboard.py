"""
Web-based keyboard input handler for teleoperation control.
This module provides keyboard control through the web UI, without requiring X11.
Receives keyboard input via HTTP API from the browser.
"""

import asyncio
import numpy as np
import logging
import time
from typing import Dict, Optional

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig, POS_STEP, ANGLE_STEP, WRIST_ROLL_INDEX, WRIST_FLEX_INDEX

logger = logging.getLogger(__name__)


class WebKeyboardHandler(BaseInputProvider):
    """Web-based keyboard input provider for dual-arm teleoperation.

    This handler processes keyboard input received from the web UI via HTTP.
    Works in headless/SSH environments without requiring X11.
    """

    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config

        # Reference to robot interface (will be set by control loop)
        self.robot_interface = None

        # Callback for disconnect command (will be set by TelegripSystem)
        self.disconnect_callback = None

        # Control state for both arms (VR-like behavior)
        self.left_arm_state = {
            "origin_position": None,
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),
            "current_wrist_roll_offset": 0.0,
            "current_wrist_flex_offset": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False,
            "last_key_time": 0.0,
            "any_key_pressed": False
        }

        self.right_arm_state = {
            "origin_position": None,
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),
            "current_wrist_roll_offset": 0.0,
            "current_wrist_flex_offset": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False,
            "last_key_time": 0.0,
            "any_key_pressed": False
        }

        # Idle timeout for repositioning target (in seconds)
        self.idle_timeout = 1.0

        # Control loop task
        self._control_task = None

    def set_robot_interface(self, robot_interface):
        """Set reference to robot interface for getting current positions."""
        self.robot_interface = robot_interface

    @property
    def is_enabled(self) -> bool:
        """Check if web keyboard control is enabled."""
        return self.is_running

    async def start(self):
        """Start the web keyboard handler."""
        self.is_running = True

        # Start control loop
        self._control_task = asyncio.create_task(self._control_loop())

        logger.info("Web keyboard handler started (no X11 required)")

    async def stop(self):
        """Stop the web keyboard handler."""
        self.is_running = False

        if self._control_task:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass

        logger.info("Web keyboard handler stopped")

    def _set_keyboard_origin(self, arm: str):
        """Set origin position for keyboard control (like VR grip press)."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if self.robot_interface:
            try:
                current_position = self.robot_interface.get_current_end_effector_position(arm)
                current_angles = self.robot_interface.get_arm_angles(arm)

                arm_state["origin_position"] = current_position.copy()
                arm_state["origin_wrist_roll"] = current_angles[WRIST_ROLL_INDEX]
                arm_state["origin_wrist_flex"] = current_angles[WRIST_FLEX_INDEX]

                # Reset current offsets
                arm_state["current_offset"] = np.zeros(3)
                arm_state["current_wrist_roll_offset"] = 0.0
                arm_state["current_wrist_flex_offset"] = 0.0

                logger.info(f"🌐 {arm.upper()} arm web keyboard origin set at position: {current_position.round(3)}")

                # Send reset signal to control loop
                reset_goal = ControlGoal(
                    arm=arm,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=None,
                    metadata={
                        "source": f"web_keyboard_grip_reset_{arm}",
                        "reset_target_to_current": True
                    }
                )
                try:
                    self.command_queue.put_nowait(reset_goal)
                except:
                    pass

            except Exception as e:
                logger.error(f"Failed to set web keyboard origin for {arm} arm: {e}")

    def _update_key_activity(self, arm: str, is_movement_key: bool = True):
        """Update the last key activity time for an arm."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        if is_movement_key:
            arm_state["last_key_time"] = time.time()
            arm_state["any_key_pressed"] = True

    def _auto_activate_arm_if_needed(self, arm: str):
        """Automatically activate position control for an arm if it's not already active."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if not arm_state["position_control_active"]:
            arm_state["position_control_active"] = True
            logger.info(f"{arm.upper()} arm position control: AUTO-ACTIVATED (web)")
            self._send_mode_change_goal(arm)
            self._set_keyboard_origin(arm)

    def on_key_press(self, key: str):
        """Handle key press events from web UI."""
        try:
            # LEFT ARM CONTROLS (WASD + QE)
            if key == 'w':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = -POS_STEP
            elif key == 's':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = POS_STEP
            elif key == 'a':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = POS_STEP
            elif key == 'd':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = -POS_STEP
            elif key == 'q':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = -POS_STEP
            elif key == 'e':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = POS_STEP

            # Left wrist roll
            elif key == 'z':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = -ANGLE_STEP
            elif key == 'x':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = ANGLE_STEP

            # Left wrist flex (pitch)
            elif key == 'r':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = -ANGLE_STEP
            elif key == 't':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = ANGLE_STEP

            # Left gripper control
            elif key == 'f':
                self.left_arm_state["gripper_closed"] = not self.left_arm_state["gripper_closed"]
                logger.info(f"LEFT gripper: {'CLOSED' if self.left_arm_state['gripper_closed'] else 'OPENED'} (web)")
                self._send_gripper_goal("left")

            # RIGHT ARM CONTROLS (UIOJKL)
            elif key == 'i':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = -POS_STEP
            elif key == 'k':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = POS_STEP
            elif key == 'j':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = POS_STEP
            elif key == 'l':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = -POS_STEP
            elif key == 'u':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = -POS_STEP
            elif key == 'o':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = POS_STEP

            # Right wrist roll
            elif key == 'n':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = -ANGLE_STEP
            elif key == 'm':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = ANGLE_STEP

            # Right wrist flex (pitch)
            elif key == 'h':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = -ANGLE_STEP
            elif key == 'y':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = ANGLE_STEP

            # Right gripper control
            elif key == ';':
                self.right_arm_state["gripper_closed"] = not self.right_arm_state["gripper_closed"]
                logger.info(f"RIGHT gripper: {'CLOSED' if self.right_arm_state['gripper_closed'] else 'OPENED'} (web)")
                self._send_gripper_goal("right")

            # Special keys
            elif key == 'tab':
                self.left_arm_state["position_control_active"] = not self.left_arm_state["position_control_active"]
                logger.info(f"LEFT arm position control: {'ACTIVATED' if self.left_arm_state['position_control_active'] else 'DEACTIVATED'} (web)")
                self._send_mode_change_goal("left")
            elif key == 'enter':
                self.right_arm_state["position_control_active"] = not self.right_arm_state["position_control_active"]
                logger.info(f"RIGHT arm position control: {'ACTIVATED' if self.right_arm_state['position_control_active'] else 'DEACTIVATED'} (web)")
                self._send_mode_change_goal("right")
            elif key == 'esc':
                logger.info("ESC pressed via web - disconnecting robot")
                if self.disconnect_callback:
                    self.disconnect_callback()

        except Exception as e:
            logger.error(f"Error handling web key press '{key}': {e}")

    def on_key_release(self, key: str):
        """Handle key release events from web UI."""
        try:
            # LEFT ARM - Reset deltas on key release
            if key in ('w', 's'):
                self.left_arm_state["delta_pos"][1] = 0
                self._check_if_all_keys_released("left")
            elif key in ('a', 'd'):
                self.left_arm_state["delta_pos"][0] = 0
                self._check_if_all_keys_released("left")
            elif key in ('q', 'e'):
                self.left_arm_state["delta_pos"][2] = 0
                self._check_if_all_keys_released("left")
            elif key in ('z', 'x'):
                self.left_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("left")
            elif key in ('r', 't'):
                self.left_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("left")

            # RIGHT ARM - Reset deltas on key release
            elif key in ('i', 'k'):
                self.right_arm_state["delta_pos"][1] = 0
                self._check_if_all_keys_released("right")
            elif key in ('j', 'l'):
                self.right_arm_state["delta_pos"][0] = 0
                self._check_if_all_keys_released("right")
            elif key in ('u', 'o'):
                self.right_arm_state["delta_pos"][2] = 0
                self._check_if_all_keys_released("right")
            elif key in ('n', 'm'):
                self.right_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("right")
            elif key in ('h', 'y'):
                self.right_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("right")

        except Exception as e:
            logger.error(f"Error handling web key release '{key}': {e}")

    def _check_if_all_keys_released(self, arm: str):
        """Check if all movement keys for an arm have been released."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if (np.all(arm_state["delta_pos"] == 0) and
            arm_state["delta_wrist_roll"] == 0 and
            arm_state["delta_wrist_flex"] == 0):
            arm_state["any_key_pressed"] = False

    def _send_gripper_goal(self, arm: str):
        """Send gripper control goal to queue."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        goal = ControlGoal(
            arm=arm,
            mode=ControlMode.IDLE,
            gripper_closed=arm_state["gripper_closed"],
            metadata={"source": f"web_keyboard_gripper_{arm}"}
        )
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass

    def _send_mode_change_goal(self, arm: str):
        """Send mode change goal to queue."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        mode = ControlMode.POSITION_CONTROL if arm_state["position_control_active"] else ControlMode.IDLE
        goal = ControlGoal(
            arm=arm,
            mode=mode,
            metadata={"source": f"web_keyboard_mode_{arm}"}
        )
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass

    def _send_idle_reset_signal(self, arm: str):
        """Send signal to reset target position due to idle timeout."""
        self._set_keyboard_origin(arm)

    async def _control_loop(self):
        """Main control loop that processes web keyboard input and sends commands."""
        logger.info("Web keyboard control loop started")

        while self.is_running:
            try:
                # Process both arms
                for arm, arm_state in [("left", self.left_arm_state), ("right", self.right_arm_state)]:
                    if arm_state["position_control_active"]:

                        # Check for idle timeout (reset origin after 1 second of inactivity)
                        current_time = time.time()
                        if (not arm_state["any_key_pressed"] and
                            arm_state["last_key_time"] > 0 and
                            current_time - arm_state["last_key_time"] >= self.idle_timeout):

                            self._send_idle_reset_signal(arm)
                            arm_state["last_key_time"] = 0

                        # Update current offsets based on deltas
                        arm_state["current_offset"] += arm_state["delta_pos"]
                        arm_state["current_wrist_roll_offset"] += arm_state["delta_wrist_roll"]
                        arm_state["current_wrist_flex_offset"] += arm_state["delta_wrist_flex"]

                        # Send position updates if there's active movement
                        if (np.any(arm_state["delta_pos"] != 0) or
                            arm_state["delta_wrist_roll"] != 0 or
                            arm_state["delta_wrist_flex"] != 0):

                            goal = ControlGoal(
                                arm=arm,
                                mode=ControlMode.POSITION_CONTROL,
                                target_position=arm_state["current_offset"].copy(),
                                wrist_roll_deg=arm_state["current_wrist_roll_offset"],
                                wrist_flex_deg=arm_state["current_wrist_flex_offset"],
                                metadata={
                                    "source": f"web_keyboard_{arm}",
                                    "relative_position": True
                                }
                            )
                            await self.send_goal(goal)

                # Control rate: 20Hz
                await asyncio.sleep(0.05)

            except Exception as e:
                logger.error(f"Error in web keyboard control loop: {e}")
                await asyncio.sleep(0.1)

        logger.info("Web keyboard control loop stopped")
