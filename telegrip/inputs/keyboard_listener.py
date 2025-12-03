"""
Keyboard input listener for teleoperation control.
Supports simultaneous dual-arm control with dedicated key layouts.
"""

import asyncio
import numpy as np
import logging
import time
from typing import Dict
import threading

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig, POS_STEP, ANGLE_STEP, GRIPPER_STEP, WRIST_ROLL_INDEX, WRIST_FLEX_INDEX

logger = logging.getLogger(__name__)


class KeyboardListener(BaseInputProvider):
    """Keyboard input provider for dual-arm teleoperation."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config

        # Reference to robot interface (will be set by control loop)
        self.robot_interface = None

        # Keyboard module (imported lazily to defer X connection)
        self._keyboard_module = None

        # Keyboard listener
        self.listener = None
        self.listener_thread = None
        
        # Control state for both arms (VR-like behavior)
        self.left_arm_state = {
            "origin_position": None,  # Like VR grip origin
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),  # Current offset from origin
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
            "origin_position": None,  # Like VR grip origin
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),  # Current offset from origin
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
    
    def set_robot_interface(self, robot_interface):
        """Set reference to robot interface for getting current positions."""
        self.robot_interface = robot_interface
    
    @property
    def is_enabled(self) -> bool:
        """Check if keyboard control is enabled."""
        return self.is_running

    async def start(self):
        """Start the keyboard listener."""
        if not self.config.enable_keyboard:
            logger.info("Keyboard listener disabled in configuration")
            return

        # Import pynput here to defer X connection requirement until actually needed
        try:
            from pynput import keyboard as pynput_keyboard
            self._keyboard_module = pynput_keyboard
        except ImportError as e:
            logger.error(f"Failed to import pynput keyboard module: {e}")
            logger.error("Keyboard control will be disabled. Make sure DISPLAY is set and X server is running.")
            return

        self.is_running = True

        # Start keyboard listener in a separate thread
        self.listener = self._keyboard_module.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        
        self.listener_thread = threading.Thread(target=self.listener.start)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        # Start control loop
        asyncio.create_task(self.control_loop())
        
        logger.info("Keyboard listener started")
        self.print_controls()
    
    async def stop(self):
        """Stop the keyboard listener."""
        self.is_running = False
        
        if self.listener:
            self.listener.stop()
        
        logger.info("Keyboard listener stopped")
    
    def print_controls(self):
        """Print keyboard control instructions."""
        # Only show detailed controls if INFO level or more verbose
        if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
            logger.info("\n" + "="*60)
            logger.info("DUAL-ARM KEYBOARD TELEOPERATION CONTROLS")
            logger.info("="*60)
            logger.info("LEFT ARM (WASD + QE):")
            logger.info("  W/S: Move Forward/Backward")
            logger.info("  A/D: Move Left/Right")
            logger.info("  Q/E: Move Down/Up")
            logger.info("  Z/X: Wrist Roll")
            logger.info("  R/T: Wrist Flex (Pitch)")
            logger.info("  F: Toggle Left Gripper Open/Closed")
            logger.info("  Tab: Manual Toggle Left Arm Position Control On/Off")
            logger.info("")
            logger.info("RIGHT ARM (UIOJKL):")
            logger.info("  I/K: Move Forward/Backward")
            logger.info("  J/L: Move Left/Right")
            logger.info("  U/O: Move Up/Down")
            logger.info("  N/M: Wrist Roll")
            logger.info("  H/Y: Wrist Flex (Pitch)")
            logger.info("  ; (semicolon): Toggle Right Gripper Open/Closed")
            logger.info("  Enter: Manual Toggle Right Arm Position Control On/Off")
            logger.info("")
            logger.info("Global:")
            logger.info("  ESC: Exit")
            logger.info("")
            logger.info("Note: Position control is automatically activated when you press")
            logger.info("      movement keys. Idle for 1 second to reset target position.")
            logger.info("="*60)
            logger.info(f"Left arm position control: {'ACTIVE' if self.left_arm_state['position_control_active'] else 'INACTIVE'}")
            logger.info(f"Right arm position control: {'ACTIVE' if self.right_arm_state['position_control_active'] else 'INACTIVE'}")
        else:
            # In quiet mode, just show that keyboard is ready
            pass

    def _set_keyboard_origin(self, arm: str):
        """Set origin position for keyboard control (like VR grip press)."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        
        if self.robot_interface:
            try:
                # Get current robot position and angles as origin
                current_position = self.robot_interface.get_current_end_effector_position(arm)
                current_angles = self.robot_interface.get_arm_angles(arm)
                
                arm_state["origin_position"] = current_position.copy()
                arm_state["origin_wrist_roll"] = current_angles[WRIST_ROLL_INDEX]
                arm_state["origin_wrist_flex"] = current_angles[WRIST_FLEX_INDEX]
                
                # Reset current offsets
                arm_state["current_offset"] = np.zeros(3)
                arm_state["current_wrist_roll_offset"] = 0.0
                arm_state["current_wrist_flex_offset"] = 0.0
                
                logger.info(f"🎮 {arm.upper()} arm keyboard origin set at position: {current_position.round(3)}")
                
                # Send reset signal to control loop (like VR grip press)
                reset_goal = ControlGoal(
                    arm=arm,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_position=None,  # Special signal
                    metadata={
                        "source": f"keyboard_grip_reset_{arm}",
                        "reset_target_to_current": True  # Signal to reset target to current position
                    }
                )
                try:
                    self.command_queue.put_nowait(reset_goal)
                except:
                    pass  # Queue might be full, ignore
                
            except Exception as e:
                logger.error(f"Failed to set keyboard origin for {arm} arm: {e}")
    
    def _update_key_activity(self, arm: str, is_movement_key: bool = True):
        """Update the last key activity time for an arm."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        if is_movement_key:
            arm_state["last_key_time"] = time.time()
            arm_state["any_key_pressed"] = True

    def on_press(self, key):
        """Handle key press events."""
        try:
            # LEFT ARM CONTROLS (WASD + QE) - Fixed W/S direction
            if key.char == 'w':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = -POS_STEP   # Forward (reversed sign)
            elif key.char == 's':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = POS_STEP    # Backward (reversed sign)
            elif key.char == 'a':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = POS_STEP    # Left (X axis)
            elif key.char == 'd':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = -POS_STEP   # Right (X axis)
            elif key.char == 'q':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = -POS_STEP   # Down (-Z)
            elif key.char == 'e':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = POS_STEP    # Up (+Z)
            
            # RIGHT ARM CONTROLS (UIOJKL) - Fixed direction signs
            elif key.char == 'i':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = -POS_STEP  # Forward (fixed sign)
            elif key.char == 'k':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = POS_STEP   # Backward (fixed sign)
            elif key.char == 'j':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = POS_STEP   # Left (X axis)
            elif key.char == 'l':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = -POS_STEP  # Right (X axis)
            elif key.char == 'u':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = -POS_STEP  # Up (fixed sign)
            elif key.char == 'o':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = POS_STEP   # Down (fixed sign)
            
            # Left gripper control
            elif key.char == 'f':
                self.left_arm_state["gripper_closed"] = not self.left_arm_state["gripper_closed"]
                logger.info(f"LEFT gripper: {'CLOSED' if self.left_arm_state['gripper_closed'] else 'OPENED'}")
                self._send_gripper_goal("left")
            
            # Left wrist roll
            elif key.char == 'z':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = -ANGLE_STEP  # CCW
            elif key.char == 'x':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = ANGLE_STEP   # CW
            
            # Left wrist flex (pitch)
            elif key.char == 'r':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = -ANGLE_STEP  # Flex down
            elif key.char == 't':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = ANGLE_STEP   # Flex up
            
            # Right wrist roll
            elif key.char == 'n':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = -ANGLE_STEP  # CCW
            elif key.char == 'm':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = ANGLE_STEP   # CW
            
            # Right wrist flex (pitch)
            elif key.char == 'h':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = -ANGLE_STEP  # Flex down
            elif key.char == 'y':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = ANGLE_STEP   # Flex up
            
            # Right gripper control
            elif key.char == ';':
                self.right_arm_state["gripper_closed"] = not self.right_arm_state["gripper_closed"]
                logger.info(f"RIGHT gripper: {'CLOSED' if self.right_arm_state['gripper_closed'] else 'OPENED'}")
                self._send_gripper_goal("right")
        
        except AttributeError:
            # Special keys
            if key == self._keyboard_module.Key.tab:
                # Toggle left arm position control
                self.left_arm_state["position_control_active"] = not self.left_arm_state["position_control_active"]
                
                if self.left_arm_state["position_control_active"]:
                    logger.info("LEFT arm position control: ACTIVATED")
                else:
                    logger.info("LEFT arm position control: DEACTIVATED")
                
                self._send_mode_change_goal("left")
            elif key == self._keyboard_module.Key.enter:
                # Toggle right arm position control
                self.right_arm_state["position_control_active"] = not self.right_arm_state["position_control_active"]
                
                if self.right_arm_state["position_control_active"]:
                    logger.info("RIGHT arm position control: ACTIVATED")
                else:
                    logger.info("RIGHT arm position control: DEACTIVATED")
                
                self._send_mode_change_goal("right")
            elif key == self._keyboard_module.Key.esc:
                logger.info("ESC pressed. Stopping keyboard control.")
                self.is_running = False
                return False  # Stop the listener
    
    def on_release(self, key):
        """Handle key release events."""
        try:
            # LEFT ARM - Reset deltas on key release (Y axis for W/S, X axis for A/D)
            if key.char in ('w', 's'):
                self.left_arm_state["delta_pos"][1] = 0  # Forward/Back (Y axis)
                self._check_if_all_keys_released("left")
            elif key.char in ('a', 'd'):
                self.left_arm_state["delta_pos"][0] = 0  # Left/Right (X axis)
                self._check_if_all_keys_released("left")
            elif key.char in ('q', 'e'):
                self.left_arm_state["delta_pos"][2] = 0
                self._check_if_all_keys_released("left")
            elif key.char in ('z', 'x'):
                self.left_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("left")
            elif key.char in ('r', 't'):
                self.left_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("left")
            
            # RIGHT ARM - Reset deltas on key release (swapped I/K and J/L axes)
            elif key.char in ('i', 'k'):
                self.right_arm_state["delta_pos"][1] = 0  # Forward/Back (Y axis)
                self._check_if_all_keys_released("right")
            elif key.char in ('j', 'l'):
                self.right_arm_state["delta_pos"][0] = 0  # Left/Right (X axis)
                self._check_if_all_keys_released("right")
            elif key.char in ('u', 'o'):
                self.right_arm_state["delta_pos"][2] = 0  # Up/Down
                self._check_if_all_keys_released("right")
            elif key.char in ('n', 'm'):
                self.right_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("right")
            elif key.char in ('h', 'y'):
                self.right_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("right")
        except AttributeError:
            # Handle special keys if needed (currently none for the new layout)
            pass

    def _check_if_all_keys_released(self, arm: str):
        """Check if all movement keys for an arm have been released."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        
        # Check if all deltas are zero
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
            metadata={"source": f"keyboard_gripper_{arm}"}
        )
        # Put goal in queue (non-blocking)
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass  # Queue might be full, ignore
    
    def _send_mode_change_goal(self, arm: str):
        """Send mode change goal to queue."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        mode = ControlMode.POSITION_CONTROL if arm_state["position_control_active"] else ControlMode.IDLE
        goal = ControlGoal(
            arm=arm,
            mode=mode,
            metadata={"source": f"keyboard_mode_{arm}"}
        )
        # Put goal in queue (non-blocking)
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass  # Queue might be full, ignore

    def _send_idle_reset_signal(self, arm: str):
        """Send signal to control loop to reset target position due to idle timeout."""
        # This acts like "grip release" - reset origin for next movement
        self._set_keyboard_origin(arm)
    
    async def control_loop(self):
        """Main control loop that processes keyboard input and sends commands."""
        logger.info("Dual-arm keyboard control loop started")
        
        while self.is_running:
            try:
                # Process both arms
                for arm, arm_state in [("left", self.left_arm_state), ("right", self.right_arm_state)]:
                    if arm_state["position_control_active"]:
                        
                        # Check if we should reset origin (after 1 second of inactivity) - like VR grip release/press
                        current_time = time.time()
                        if (not arm_state["any_key_pressed"] and 
                            arm_state["last_key_time"] > 0 and 
                            current_time - arm_state["last_key_time"] >= self.idle_timeout):
                            
                            # Reset origin like "grip release + grip press"
                            self._send_idle_reset_signal(arm)
                            # Reset the timer to prevent continuous signaling
                            arm_state["last_key_time"] = 0
                        
                        # Update current offsets based on deltas (like VR accumulates from grip origin)
                        arm_state["current_offset"] += arm_state["delta_pos"]
                        arm_state["current_wrist_roll_offset"] += arm_state["delta_wrist_roll"]
                        arm_state["current_wrist_flex_offset"] += arm_state["delta_wrist_flex"]
                        
                        # Send absolute offset from origin if there's movement or wrist rotation
                        if (np.any(arm_state["delta_pos"] != 0) or 
                            arm_state["delta_wrist_roll"] != 0 or 
                            arm_state["delta_wrist_flex"] != 0):
                            
                            # Send absolute offset from origin (like VR)
                            goal = ControlGoal(
                                arm=arm,
                                mode=ControlMode.POSITION_CONTROL,
                                target_position=arm_state["current_offset"].copy(),  # Absolute offset from origin
                                wrist_roll_deg=arm_state["current_wrist_roll_offset"],   # Absolute offset from origin
                                wrist_flex_deg=arm_state["current_wrist_flex_offset"],   # Absolute offset from origin
                                metadata={
                                    "source": f"keyboard_{arm}",
                                    "relative_position": True  # Same as VR - absolute offset from origin
                                }
                            )
                            await self.send_goal(goal)
                
                # Sleep to control update rate
                await asyncio.sleep(0.05)  # 20Hz update rate
                
            except Exception as e:
                logger.error(f"Error in keyboard control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Dual-arm keyboard control loop stopped")

    def _auto_activate_arm_if_needed(self, arm: str):
        """Automatically activate position control for an arm if it's not already active."""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        
        if not arm_state["position_control_active"]:
            # Activate position control
            arm_state["position_control_active"] = True
            logger.info(f"{arm.upper()} arm position control: AUTO-ACTIVATED")
            self._send_mode_change_goal(arm)
            
            # Set origin like "grip press"
            self._set_keyboard_origin(arm)

    def get_idle_status(self, arm: str) -> tuple:
        """Get idle status for an arm (used by control loop for 1-second reset logic).
        
        Returns:
            tuple: (is_idle, time_since_last_key)
        """
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        
        if not arm_state["position_control_active"]:
            return False, 0
        
        current_time = time.time()
        
        # Check if idle (no keys pressed and some time has passed since last key)
        is_idle = (not arm_state["any_key_pressed"] and 
                  arm_state["last_key_time"] > 0)
        
        time_since_last = (current_time - arm_state["last_key_time"]) if arm_state["last_key_time"] > 0 else 0
        
        return is_idle, time_since_last 