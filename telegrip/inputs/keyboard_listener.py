"""
Keyboard input listener for teleoperation control.
Supports simultaneous dual-arm control with dedicated key layouts.
"""

import asyncio
import numpy as np
import logging
from typing import Dict
from pynput import keyboard
import threading

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig, POS_STEP, ANGLE_STEP, GRIPPER_STEP

logger = logging.getLogger(__name__)


class KeyboardListener(BaseInputProvider):
    """Keyboard input provider for dual-arm teleoperation."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        
        # Reference to robot interface (will be set by control loop)
        self.robot_interface = None
        
        # Keyboard listener
        self.listener = None
        self.listener_thread = None
        
        # Control state for both arms
        self.left_arm_state = {
            "target_position": None,  # Will be initialized from current robot position
            "target_wrist_roll": 0.0,
            "target_wrist_flex": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False
        }
        
        self.right_arm_state = {
            "target_position": None,  # Will be initialized from current robot position
            "target_wrist_roll": 0.0,
            "target_wrist_flex": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False
        }
    
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
        
        self.is_running = True
        
        # Start keyboard listener in a separate thread
        self.listener = keyboard.Listener(
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
            logger.info("      movement keys. Tab/Enter are only needed for manual toggle.")
            logger.info("="*60)
            logger.info(f"Left arm position control: {'ACTIVE' if self.left_arm_state['position_control_active'] else 'INACTIVE'}")
            logger.info(f"Right arm position control: {'ACTIVE' if self.right_arm_state['position_control_active'] else 'INACTIVE'}")
        else:
            # In quiet mode, just show that keyboard is ready
            pass
    
    def _initialize_arm_position(self, arm: str):
        """Initialize target position from current robot position when activating control."""
        if not self.robot_interface:
            # Fallback to default position if no robot interface
            default_position = np.array([0.2, 0.0, 0.15])
            logger.warning(f"No robot interface available, using default position for {arm} arm: {default_position}")
            return default_position
        
        try:
            # Get current end effector position
            current_position = self.robot_interface.get_current_end_effector_position(arm)
            logger.info(f"Initialized {arm} arm keyboard control at current position: {current_position.round(3)}")
            return current_position.copy()
        except Exception as e:
            # Fallback to default position on error
            default_position = np.array([0.2, 0.0, 0.15])
            logger.warning(f"Failed to get current {arm} arm position: {e}. Using default: {default_position}")
            return default_position
    
    def _initialize_arm_wrist_roll(self, arm: str):
        """Initialize target wrist roll from current robot angle when activating control."""
        if not self.robot_interface:
            logger.warning(f"No robot interface available, using default wrist roll for {arm} arm: 0.0°")
            return 0.0
        
        try:
            # Get current wrist roll angle
            current_angles = self.robot_interface.get_arm_angles(arm)
            from ..config import WRIST_ROLL_INDEX
            current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
            logger.info(f"Initialized {arm} arm wrist roll at current angle: {current_wrist_roll:.1f}°")
            return current_wrist_roll
        except Exception as e:
            logger.warning(f"Failed to get current {arm} arm wrist roll: {e}. Using default: 0.0°")
            return 0.0
    
    def _initialize_arm_wrist_flex(self, arm: str):
        """Initialize target wrist flex from current robot angle when activating control."""
        if not self.robot_interface:
            logger.warning(f"No robot interface available, using default wrist flex for {arm} arm: 0.0°")
            return 0.0
        
        try:
            # Get current wrist flex angle
            current_angles = self.robot_interface.get_arm_angles(arm)
            from ..config import WRIST_FLEX_INDEX
            current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
            logger.info(f"Initialized {arm} arm wrist flex at current angle: {current_wrist_flex:.1f}°")
            return current_wrist_flex
        except Exception as e:
            logger.warning(f"Failed to get current {arm} arm wrist flex: {e}. Using default: 0.0°")
            return 0.0
    
    def on_press(self, key):
        """Handle key press events."""
        try:
            # LEFT ARM CONTROLS (WASD + QE) - Fixed W/S direction
            if key.char == 'w':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][1] = -POS_STEP   # Forward (reversed sign)
            elif key.char == 's':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][1] = POS_STEP    # Backward (reversed sign)
            elif key.char == 'a':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][0] = POS_STEP    # Left (X axis)
            elif key.char == 'd':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][0] = -POS_STEP   # Right (X axis)
            elif key.char == 'q':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][2] = -POS_STEP   # Down (-Z)
            elif key.char == 'e':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_pos"][2] = POS_STEP    # Up (+Z)
            
            # RIGHT ARM CONTROLS (UIOJKL) - Fixed direction signs
            elif key.char == 'i':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][1] = -POS_STEP  # Forward (fixed sign)
            elif key.char == 'k':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][1] = POS_STEP   # Backward (fixed sign)
            elif key.char == 'j':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][0] = POS_STEP   # Left (X axis)
            elif key.char == 'l':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][0] = -POS_STEP  # Right (X axis)
            elif key.char == 'u':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][2] = -POS_STEP  # Up (fixed sign)
            elif key.char == 'o':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_pos"][2] = POS_STEP   # Down (fixed sign)
            
            # Left gripper control
            elif key.char == 'f':
                self.left_arm_state["gripper_closed"] = not self.left_arm_state["gripper_closed"]
                logger.info(f"LEFT gripper: {'CLOSED' if self.left_arm_state['gripper_closed'] else 'OPENED'}")
                self._send_gripper_goal("left")
            
            # Left wrist roll
            elif key.char == 'z':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_wrist_roll"] = -ANGLE_STEP  # CCW
            elif key.char == 'x':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_wrist_roll"] = ANGLE_STEP   # CW
            
            # Left wrist flex (pitch)
            elif key.char == 'r':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_wrist_flex"] = -ANGLE_STEP  # Flex down
            elif key.char == 't':
                self._auto_activate_arm_if_needed("left")
                self.left_arm_state["delta_wrist_flex"] = ANGLE_STEP   # Flex up
            
            # Right wrist roll
            elif key.char == 'n':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_wrist_roll"] = -ANGLE_STEP  # CCW
            elif key.char == 'm':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_wrist_roll"] = ANGLE_STEP   # CW
            
            # Right wrist flex (pitch)
            elif key.char == 'h':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_wrist_flex"] = -ANGLE_STEP  # Flex down
            elif key.char == 'y':
                self._auto_activate_arm_if_needed("right")
                self.right_arm_state["delta_wrist_flex"] = ANGLE_STEP   # Flex up
            
            # Right gripper control
            elif key.char == ';':
                self.right_arm_state["gripper_closed"] = not self.right_arm_state["gripper_closed"]
                logger.info(f"RIGHT gripper: {'CLOSED' if self.right_arm_state['gripper_closed'] else 'OPENED'}")
                self._send_gripper_goal("right")
        
        except AttributeError:
            # Special keys
            if key == keyboard.Key.tab:
                # Toggle left arm position control
                self.left_arm_state["position_control_active"] = not self.left_arm_state["position_control_active"]
                
                if self.left_arm_state["position_control_active"]:
                    # Initialize target position from current robot position
                    self.left_arm_state["target_position"] = self._initialize_arm_position("left")
                    self.left_arm_state["target_wrist_roll"] = self._initialize_arm_wrist_roll("left")
                    self.left_arm_state["target_wrist_flex"] = self._initialize_arm_wrist_flex("left")
                    logger.info("LEFT arm position control: ACTIVATED")
                else:
                    logger.info("LEFT arm position control: DEACTIVATED")
                
                self._send_mode_change_goal("left")
            elif key == keyboard.Key.enter:
                # Toggle right arm position control
                self.right_arm_state["position_control_active"] = not self.right_arm_state["position_control_active"]
                
                if self.right_arm_state["position_control_active"]:
                    # Initialize target position from current robot position
                    self.right_arm_state["target_position"] = self._initialize_arm_position("right")
                    self.right_arm_state["target_wrist_roll"] = self._initialize_arm_wrist_roll("right")
                    self.right_arm_state["target_wrist_flex"] = self._initialize_arm_wrist_flex("right")
                    logger.info("RIGHT arm position control: ACTIVATED")
                else:
                    logger.info("RIGHT arm position control: DEACTIVATED")
                
                self._send_mode_change_goal("right")
            elif key == keyboard.Key.esc:
                logger.info("ESC pressed. Stopping keyboard control.")
                self.is_running = False
                return False  # Stop the listener
    
    def on_release(self, key):
        """Handle key release events."""
        try:
            # LEFT ARM - Reset deltas on key release (Y axis for W/S, X axis for A/D)
            if key.char in ('w', 's'):
                self.left_arm_state["delta_pos"][1] = 0  # Forward/Back (Y axis)
            elif key.char in ('a', 'd'):
                self.left_arm_state["delta_pos"][0] = 0  # Left/Right (X axis)
            elif key.char in ('q', 'e'):
                self.left_arm_state["delta_pos"][2] = 0
            elif key.char in ('z', 'x'):
                self.left_arm_state["delta_wrist_roll"] = 0
            elif key.char in ('r', 't'):
                self.left_arm_state["delta_wrist_flex"] = 0
            
            # RIGHT ARM - Reset deltas on key release (swapped I/K and J/L axes)
            elif key.char in ('i', 'k'):
                self.right_arm_state["delta_pos"][1] = 0  # Forward/Back (Y axis)
            elif key.char in ('j', 'l'):
                self.right_arm_state["delta_pos"][0] = 0  # Left/Right (X axis)
            elif key.char in ('u', 'o'):
                self.right_arm_state["delta_pos"][2] = 0  # Up/Down
            elif key.char in ('n', 'm'):
                self.right_arm_state["delta_wrist_roll"] = 0
            elif key.char in ('h', 'y'):
                self.right_arm_state["delta_wrist_flex"] = 0
        except AttributeError:
            # Handle special keys if needed (currently none for the new layout)
            pass
    
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
    
    async def control_loop(self):
        """Main control loop that processes keyboard input and sends commands."""
        logger.info("Dual-arm keyboard control loop started")
        
        while self.is_running:
            try:
                # Process both arms
                for arm, arm_state in [("left", self.left_arm_state), ("right", self.right_arm_state)]:
                    if arm_state["position_control_active"] and arm_state["target_position"] is not None:
                        # Update target state based on deltas
                        arm_state["target_position"] += arm_state["delta_pos"]
                        arm_state["target_wrist_roll"] += arm_state["delta_wrist_roll"]
                        arm_state["target_wrist_flex"] += arm_state["delta_wrist_flex"]
                        
                        # Send position control goal if there's movement or wrist rotation
                        if (np.any(arm_state["delta_pos"] != 0) or 
                            arm_state["delta_wrist_roll"] != 0 or 
                            arm_state["delta_wrist_flex"] != 0):
                            goal = ControlGoal(
                                arm=arm,
                                mode=ControlMode.POSITION_CONTROL,
                                target_position=arm_state["target_position"].copy(),
                                wrist_roll_deg=arm_state["target_wrist_roll"],
                                wrist_flex_deg=arm_state["target_wrist_flex"],
                                metadata={
                                    "source": f"keyboard_{arm}",
                                    "relative_position": False
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
            arm_state["target_position"] = self._initialize_arm_position(arm)
            arm_state["target_wrist_roll"] = self._initialize_arm_wrist_roll(arm)
            arm_state["target_wrist_flex"] = self._initialize_arm_wrist_flex(arm)
            logger.info(f"{arm.upper()} arm position control: AUTO-ACTIVATED")
            self._send_mode_change_goal(arm) 