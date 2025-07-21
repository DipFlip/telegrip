"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
from .drawing.drawing_manager import DrawingManager

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None  # For visualization
        self.origin_position = None  # Robot position when grip was activated
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        
    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        self.keyboard_listener = None  # Reference to keyboard listener for commands
        
        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # Drawing manager
        self.drawing_manager = DrawingManager()
        self.drawing_in_progress = False
        
        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second
        
        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False
        
        self.is_running = False
    
    async def setup(self) -> bool:
        """Setup robot interface and visualizer."""
        success = True
        setup_errors = []
        
        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # Setup PyBullet simulation, IK and visualizer
        if self.config.enable_pybullet:
            try:
                # Import PyBulletVisualizer on demand
                from .core.visualizer import PyBulletVisualizer
                
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_path(), 
                    use_gui=self.config.enable_pybullet_gui,
                    log_level=self.config.log_level
                )
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    # Connect kinematics to robot interface
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,  # Pass both robot instances
                        self.visualizer.joint_indices,  # Pass both joint index mappings
                        self.visualizer.end_effector_link_indices,  # Pass both end effector indices
                        joint_limits_min,
                        joint_limits_max
                    )
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
        
        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # Set robot interface on keyboard listener so it can get current positions
        if self.keyboard_listener and self.robot_interface:
            self.keyboard_listener.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on keyboard listener")
        
        return success
    
    async def start(self):
        """Start the control loop."""
        if not await self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # Initialize arm states with current robot positions
        self._initialize_arm_states()
        
        # Main control loop
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()
                
                # Update robot (with error resilience)
                self._update_robot_safely()
                
                # Update visualization
                if self.visualizer:
                    self._update_visualization()
                
                # Periodic logging
                self._periodic_logging()
                
                # Control rate
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Control loop stopped")
    
    async def stop(self):
        """Stop the control loop."""
        self.is_running = False
        
        # Cleanup
        if self.robot_interface:
            self.robot_interface.disconnect()
        
        if self.visualizer:
            self.visualizer.disconnect()
    
    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            # Get current end effector positions
            left_pos = self.robot_interface.get_current_end_effector_position("left")
            right_pos = self.robot_interface.get_current_end_effector_position("right")
            
            # Initialize target positions to current positions (ensure deep copies)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()
            
            # Get current wrist roll angles
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[WRIST_FLEX_INDEX]
            
            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process regular control goals
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")
        
        if action == 'enable_keyboard':
            if self.keyboard_listener:
                await self.keyboard_listener.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == 'disable_keyboard':
            if self.keyboard_listener:
                await self.keyboard_listener.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == 'web_keypress':
            # Handle individual keypress events from web interface
            if self.keyboard_listener and self.keyboard_listener.is_enabled:
                key = command.get('key')
                event = command.get('event')  # 'press' or 'release'
                logger.info(f"🎮 Processing web keypress: {key}_{event}")
                await self._handle_web_keypress(key, event)
            else:
                logger.warning("🎮 Keyboard control not enabled for web keypress")
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # No need to sync keyboard targets - unified system handles this automatically
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")
                    
                    # Hide visualization markers
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")
    
    async def _handle_web_keypress(self, key: str, event: str):
        """Handle keypress events from web interface by delegating to keyboard listener."""
        if not self.keyboard_listener:
            return
            
        # Create a mock key object for compatibility with keyboard listener
        class MockKey:
            def __init__(self, char=None, key_type=None):
                self.char = char
                self.key_type = key_type
        
        # Map special keys
        special_keys = {
            'tab': 'Key.tab',
            'enter': 'Key.enter', 
            'esc': 'Key.esc'
        }
        
        try:
            if event == 'press':
                if key in special_keys:
                    # Handle special keys
                    if key == 'tab':
                        from pynput.keyboard import Key
                        mock_key = Key.tab
                    elif key == 'enter':
                        from pynput.keyboard import Key
                        mock_key = Key.enter
                    elif key == 'esc':
                        from pynput.keyboard import Key
                        mock_key = Key.esc
                    else:
                        return
                else:
                    # Handle character keys
                    mock_key = MockKey(char=key)
                
                # Call keyboard listener's on_press method
                self.keyboard_listener.on_press(mock_key)
                
            elif event == 'release':
                if key in special_keys:
                    # Special keys don't typically need release handling
                    return
                else:
                    # Handle character key release
                    mock_key = MockKey(char=key)
                
                # Call keyboard listener's on_release method
                self.keyboard_listener.on_release(mock_key)
                
        except Exception as e:
            logger.error(f"Error handling web keypress {key}_{event}: {e}")
    
    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # Prevent VR interference during drawing
        if (self.drawing_in_progress and 
            goal.metadata and 
            goal.metadata.get("source") in ["vr_grip_release", "vr_trigger_release"] and
            goal.arm == self.drawing_manager.current_drawing_arm):
            logger.warning(f"🎨 Ignoring VR command during drawing: {goal.metadata.get('source')}")
            return
        
        # Handle special reset signal from keyboard idle timeout
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                # Reset target position to current robot position
                current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                
                arm_state.target_position = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.origin_position = current_position.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔄 {goal.arm.upper()} arm: Target position reset to current robot position (idle timeout)")
            return
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # Activate position control - always reset target to current position
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                if self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    
                    # Reset everything to current position (like VR grip press)
                    arm_state.target_position = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.origin_position = current_position.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED (target reset to current position)")
                
            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control
                arm_state.reset()
                
                # Hide visualization markers
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # Handle position control - both VR and keyboard now work the same way (absolute offset from origin)
        # SAFETY: Skip all position control for test moves to prevent dangerous robot movements
        if (goal.target_position is not None and 
            arm_state.mode == ControlMode.POSITION_CONTROL and
            not (goal.metadata and goal.metadata.get("source") == "test_move")):
            if goal.metadata and goal.metadata.get("relative_position", False):
                # Check if we need to reset origin to current position first (for test moves)
                if goal.metadata.get("reset_origin_to_current", False) and self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    arm_state.origin_position = current_position.copy()
                    logger.info(f"🧪 {goal.arm.upper()} origin reset to current position: {arm_state.origin_position.round(3)}")
                
                # Both VR and keyboard send absolute offset from robot origin position
                if arm_state.origin_position is not None:
                    old_target = arm_state.target_position.copy() if arm_state.target_position is not None else None
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
                    
                    logger.info(f"🧪 {goal.arm.upper()} relative position update:")
                    logger.info(f"🧪   Origin: {arm_state.origin_position.round(3)}")
                    logger.info(f"🧪   Delta: {goal.target_position.round(3)}")
                    logger.info(f"🧪   Old target: {old_target.round(3) if old_target is not None else 'None'}")
                    logger.info(f"🧪   New target: {arm_state.target_position.round(3)}")
                else:
                    # No origin set yet, use current position as base
                    if self.robot_interface:
                        current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                        arm_state.target_position = current_position + goal.target_position
                        arm_state.goal_position = arm_state.target_position.copy()
                        
                        logger.info(f"🧪 {goal.arm.upper()} relative position (no origin):")
                        logger.info(f"🧪   Current: {current_position.round(3)}")
                        logger.info(f"🧪   Delta: {goal.target_position.round(3)}")
                        logger.info(f"🧪   New target: {arm_state.target_position.round(3)}")
            else:
                # Absolute position (legacy - should not be used anymore)
                old_target = arm_state.target_position.copy() if arm_state.target_position is not None else None
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()
                
                logger.info(f"🧪 {goal.arm.upper()} absolute position update:")
                logger.info(f"🧪   Old target: {old_target.round(3) if old_target is not None else 'None'}")
                logger.info(f"🧪   New target: {arm_state.target_position.round(3)}")
            
            # Handle wrist movements - both VR and keyboard send absolute offsets from origin
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    # Absolute wrist roll (legacy)
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
            
            # Handle wrist flex - both VR and keyboard send absolute offsets from origin
            if goal.wrist_flex_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    # Absolute wrist flex (legacy)
                    arm_state.current_wrist_flex = goal.wrist_flex_deg
        
        # Handle drawing commands
        if goal.metadata and (goal.metadata.get("source") == "vr_a_button" or goal.metadata.get("source") == "web_ui"):
            await self._handle_drawing_command(goal)
        
        # Handle test move commands - execute them as smooth animated movements ONLY
        if goal.metadata and goal.metadata.get("source") == "test_move":
            logger.info(f"🧪 Processing smooth test move: {goal.metadata.get('action')} for {goal.arm} arm")
            logger.info(f"🧪 Target delta: {goal.target_position}")
            # Set flag to enable IK logging for test moves
            self._test_move_active = True
            
            # Execute as smooth animated movement - bypass all regular position control
            asyncio.create_task(self._execute_smooth_test_move(goal))
            return  # SKIP all regular position control processing
        
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)
    
    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues
    
    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return
        
        # Update left arm (only if connected)
        if (self.left_arm.mode == ControlMode.POSITION_CONTROL and 
            self.left_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("left")):
            
            logger.debug(f"🎨 LEFT IK: target={self.left_arm.target_position.round(3)}")
            
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_position)
            
            logger.debug(f"🎨 LEFT IK solution: {ik_solution.round(1)}")
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("left", ik_solution, 
                                                 self.left_arm.current_wrist_flex, 
                                                 self.left_arm.current_wrist_roll, 
                                                 current_gripper)

        # Update right arm (only if connected)
        if (self.right_arm.mode == ControlMode.POSITION_CONTROL and 
            self.right_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("right")):
            
            # Log IK during drawing or when test move is active
            log_ik = self.drawing_in_progress or (hasattr(self, '_test_move_active') and self._test_move_active)
            if log_ik:
                logger.info(f"🎨 RIGHT IK: target={self.right_arm.target_position.round(3)}")
            
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_position)
            
            if log_ik:
                logger.info(f"🎨 RIGHT IK solution: {ik_solution.round(1)}")
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("right", ik_solution, 
                                                  self.right_arm.current_wrist_flex, 
                                                  self.right_arm.current_wrist_roll, 
                                                  current_gripper)

        # Send commands to robot
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            success = self.robot_interface.send_command()
            if self.drawing_in_progress:
                logger.info(f"🎨 Robot command sent: success={success}")
        elif self.drawing_in_progress:
            logger.warning(f"🎨 Robot not connected/engaged: connected={self.robot_interface.is_connected}, engaged={self.robot_interface.is_engaged}")
    
    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return
        
        # Update robot poses for both arms using ACTUAL angles from robot hardware
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # Update visualization markers
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # Step simulation
        self.visualizer.step_simulation()
    
    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    async def _handle_drawing_command(self, goal: ControlGoal):
        """Handle drawing-related commands from VR A button."""
        if not goal.metadata:
            return
            
        action = goal.metadata.get("action")
        
        if action == "drawing_calibration_point":
            # Get current robot goal position for this arm (this is what matters for drawing)
            robot_goal_position = None
            if self.robot_interface:
                arm_state = self.left_arm if goal.arm == "left" else self.right_arm
                # Use the current goal position (where the robot is trying to be)
                robot_goal_position = arm_state.goal_position.copy() if arm_state.goal_position is not None else None
                
                # If no goal position, fall back to current end effector position
                if robot_goal_position is None:
                    robot_goal_position = self.robot_interface.get_current_end_effector_position(goal.arm)
            
            # Add calibration point (VR position is just for reference, robot position is what we use)
            vr_position = goal.metadata.get("position", {})
            calibration_complete = self.drawing_manager.add_calibration_point(vr_position, robot_goal_position)
            
            if calibration_complete:
                logger.info("🎨 Plane calibration complete! Ready to draw.")
        
        elif action == "start_drawing_calibration":
            # Start calibration process
            self.drawing_manager.start_calibration(goal.arm)
        
        elif action == "plane_drawing":
            # Execute plane-based drawing (3x3cm square on calibrated plane)
            if self.drawing_manager.is_ready_to_draw():
                await self._execute_plane_drawing(goal.arm)
            else:
                logger.error("🎨 Plane drawing requires 4-point calibration first")
        
        elif action == "svg_drawing":
            # Execute SVG drawing
            filename = goal.metadata.get("filename")
            if self.drawing_manager.is_ready_to_draw():
                await self._execute_svg_drawing(goal.arm, filename)
            else:
                logger.error("🎨 SVG drawing requires 4-point calibration first")
            
    async def _execute_square_drawing(self, arm: str):
        """Just move the goal marker in PyBullet to the 4 saved positions."""
        if not self.drawing_manager.is_ready_to_draw():
            logger.error("Drawing manager not ready - calibration incomplete")
            return
            
        logger.info(f"🎨 Starting simple drawing: moving goal marker for {arm} arm...")
        
        # Get the 4 saved robot goal positions from calibration
        saved_positions = []
        for point in self.drawing_manager.calibration_points:
            if point.robot_position is not None:
                saved_positions.append(point.robot_position.copy())
        
        if len(saved_positions) != 4:
            logger.error(f"Expected 4 saved positions, got {len(saved_positions)}")
            return
        
        logger.info(f"🎨 Will move goal marker to 4 saved positions: {[pos.round(3) for pos in saved_positions]}")
        
        # Just move the goal marker directly
        asyncio.create_task(self._move_goal_marker_sequence(arm, saved_positions))
    
    async def _execute_plane_drawing(self, arm: str):
        """Draw a 3x3cm square on the center of the calibrated plane with pen up/down moves."""
        if not self.drawing_manager.is_ready_to_draw():
            logger.error("Drawing manager not ready - calibration incomplete")
            return
            
        logger.info(f"🎨 Starting plane drawing: 3x3cm square with {arm} arm...")
        
        try:
            # Calculate plane from 4 calibration points
            plane_info = self._calculate_drawing_plane()
            if not plane_info:
                logger.error("🎨 Could not calculate drawing plane")
                return
            
            # Generate 3x3cm square path with pen up/down moves
            drawing_moves = self._generate_square_drawing_moves(plane_info, square_size_cm=3.0)
            if not drawing_moves:
                logger.error("🎨 Could not generate drawing moves")
                return
            
            logger.info(f"🎨 Generated {len(drawing_moves)} drawing moves")
            
            # Execute the drawing sequence with smooth animation
            asyncio.create_task(self._execute_drawing_moves_sequence(arm, drawing_moves))
            
        except Exception as e:
            logger.error(f"🎨 Error during plane drawing: {e}")
    
    async def _execute_svg_drawing(self, arm: str, filename: str):
        """Draw an SVG file on the calibrated plane."""
        if not self.drawing_manager.is_ready_to_draw():
            logger.error("Drawing manager not ready - calibration incomplete")
            return
        
        if not filename:
            logger.error("🎨 No SVG filename provided")
            return
            
        logger.info(f"🎨 Starting SVG drawing: {filename} with {arm} arm...")
        
        try:
            # Import SVG parser
            from .drawing.svg_parser import SVGParser, SVGToRobotConverter
            from .utils import get_absolute_path
            import os
            
            # Get full path to SVG file
            svg_path = get_absolute_path(f"vector_art/{filename}")
            if not os.path.exists(svg_path):
                logger.error(f"🎨 SVG file not found: {svg_path}")
                return
            
            # Parse SVG file
            parser = SVGParser()
            svg_data = parser.parse_svg_file(svg_path)
            if not svg_data:
                logger.error(f"🎨 Failed to parse SVG file: {filename}")
                return
            
            # Calculate plane from 4 calibration points
            plane_info = self._calculate_drawing_plane()
            if not plane_info:
                logger.error("🎨 Could not calculate drawing plane for SVG")
                return
            
            # Convert SVG to robot drawing moves
            converter = SVGToRobotConverter(plane_info)
            drawing_moves = converter.convert_svg_to_drawing_moves(svg_data, target_size_cm=2.5)
            if not drawing_moves:
                logger.error("🎨 Could not generate drawing moves from SVG")
                return
            
            logger.info(f"🎨 Generated {len(drawing_moves)} moves for SVG: {filename}")
            
            # Execute the drawing sequence with smooth animation (no waiting for SVG)
            asyncio.create_task(self._execute_drawing_moves_sequence(arm, drawing_moves, wait_for_arrival=False))
            
        except Exception as e:
            logger.error(f"🎨 Error during SVG drawing: {e}")
            import traceback
            logger.error(f"🎨 Traceback: {traceback.format_exc()}")
    
    def _calculate_drawing_plane(self):
        """Calculate plane information from 4 calibration points."""
        if len(self.drawing_manager.calibration_points) != 4:
            return None
        
        # Get the 4 robot positions
        points = []
        for point in self.drawing_manager.calibration_points:
            if point.robot_position is not None:
                points.append(point.robot_position.copy())
        
        if len(points) != 4:
            return None
        
        # Assume points are [bottom-left, bottom-right, top-right, top-left]
        origin = points[0]  # bottom-left
        x_axis = points[1] - points[0]  # bottom edge vector
        y_axis = points[3] - points[0]  # left edge vector
        
        # Calculate normal vector (positive z should be "up" from plane)
        normal = np.cross(x_axis, y_axis)
        normal = normal / np.linalg.norm(normal)  # normalize
        
        # Ensure normal points "up" (positive z component)
        if normal[2] < 0:
            normal = -normal
        
        # Normalize axis vectors
        x_length = np.linalg.norm(x_axis)
        y_length = np.linalg.norm(y_axis)
        x_unit = x_axis / x_length
        y_unit = y_axis / y_length
        
        plane_info = {
            'origin': origin,
            'x_unit': x_unit,
            'y_unit': y_unit, 
            'normal': normal,
            'x_length': x_length,
            'y_length': y_length
        }
        
        logger.info(f"🎨 Plane: origin={origin.round(3)}, size={x_length:.3f}x{y_length:.3f}m")
        logger.info(f"🎨 Normal vector: {normal.round(3)}")
        logger.info(f"🎨 DEBUG: x_axis={x_axis.round(3)}, y_axis={y_axis.round(3)}")
        logger.info(f"🎨 DEBUG: x_unit={x_unit.round(3)}, y_unit={y_unit.round(3)}")
        
        return plane_info
    
    def _generate_square_drawing_moves(self, plane_info, square_size_cm=3.0):
        """Generate drawing moves for a smiley face (square outline + eyes + mouth) with pen up/down."""
        square_size_m = square_size_cm / 100.0  # Convert to meters
        pen_up_height_m = 0.02  # 2cm above plane for better visibility
        
        # Calculate center of the plane
        center_x = plane_info['x_length'] / 2.0
        center_y = plane_info['y_length'] / 2.0
        
        # Calculate square corners (centered on plane)
        half_size = square_size_m / 2.0
        square_corners_local = [
            (center_x - half_size, center_y - half_size),  # bottom-left
            (center_x + half_size, center_y - half_size),  # bottom-right  
            (center_x + half_size, center_y + half_size),  # top-right
            (center_x - half_size, center_y + half_size),  # top-left
            (center_x - half_size, center_y - half_size),  # back to start
        ]
        
        # Convert to 3D world coordinates
        def local_to_world(x_local, y_local, pen_down=True):
            # Position on plane
            world_pos = (plane_info['origin'] + 
                        x_local * plane_info['x_unit'] + 
                        y_local * plane_info['y_unit'])
            
            # Add pen height if pen is up
            if not pen_down:
                pen_offset = pen_up_height_m * plane_info['normal']
                logger.info(f"🎨 DEBUG: Pen UP - adding offset {pen_offset.round(3)} to base position {world_pos.round(3)}")
                world_pos += pen_offset
                logger.info(f"🎨 DEBUG: Final pen UP position: {world_pos.round(3)}")
            else:
                logger.info(f"🎨 DEBUG: Pen DOWN - base position: {world_pos.round(3)}")
            
            return world_pos
        
        # Generate drawing moves for a smiley face: square outline + eyes + mouth
        moves = []
        
        # 1. Draw the square outline as a continuous line
        for i, (x, y) in enumerate(square_corners_local):
            if i == 0:
                # First corner: go to start position with pen up, then pen down
                moves.append({
                    'position': local_to_world(x, y, pen_down=False),
                    'description': f'Move to start corner (pen up)',
                    'pen_down': False
                })
                moves.append({
                    'position': local_to_world(x, y, pen_down=True), 
                    'description': f'Lower pen to start square outline',
                    'pen_down': True
                })
            else:
                # Draw continuous line to each corner
                moves.append({
                    'position': local_to_world(x, y, pen_down=True),
                    'description': f'Draw to corner {i+1}',
                    'pen_down': True
                })
        
        # Lift pen after completing square outline
        moves.append({
            'position': local_to_world(square_corners_local[-1][0], square_corners_local[-1][1], pen_down=False),
            'description': 'Lift pen after square outline',
            'pen_down': False
        })
        
        # 2. Draw left eye (small dot)
        left_eye_x = center_x - square_size_m * 0.25  # 1/4 left of center
        left_eye_y = center_y + square_size_m * 0.15  # Slightly above center, inside square
        
        moves.append({
            'position': local_to_world(left_eye_x, left_eye_y, pen_down=False),
            'description': 'Move to left eye position (pen up)',
            'pen_down': False
        })
        moves.append({
            'position': local_to_world(left_eye_x, left_eye_y, pen_down=True),
            'description': 'Draw left eye dot',
            'pen_down': True
        })
        moves.append({
            'position': local_to_world(left_eye_x, left_eye_y, pen_down=False),
            'description': 'Lift pen after left eye',
            'pen_down': False
        })
        
        # 3. Draw right eye (small dot)  
        right_eye_x = center_x + square_size_m * 0.25  # 1/4 right of center
        right_eye_y = center_y + square_size_m * 0.15  # Slightly above center, inside square
        
        moves.append({
            'position': local_to_world(right_eye_x, right_eye_y, pen_down=False),
            'description': 'Move to right eye position (pen up)',
            'pen_down': False
        })
        moves.append({
            'position': local_to_world(right_eye_x, right_eye_y, pen_down=True),
            'description': 'Draw right eye dot',
            'pen_down': True
        })
        moves.append({
            'position': local_to_world(right_eye_x, right_eye_y, pen_down=False),
            'description': 'Lift pen after right eye',
            'pen_down': False
        })
        
        # 4. Draw mouth (small horizontal line)
        mouth_left_x = center_x - square_size_m * 0.2   # Left side of mouth
        mouth_right_x = center_x + square_size_m * 0.2  # Right side of mouth
        mouth_y = center_y - square_size_m * 0.25       # Below center
        
        moves.append({
            'position': local_to_world(mouth_left_x, mouth_y, pen_down=False),
            'description': 'Move to mouth start (pen up)',
            'pen_down': False
        })
        moves.append({
            'position': local_to_world(mouth_left_x, mouth_y, pen_down=True),
            'description': 'Start drawing mouth',
            'pen_down': True
        })
        moves.append({
            'position': local_to_world(mouth_right_x, mouth_y, pen_down=True),
            'description': 'Draw mouth line',
            'pen_down': True
        })
        moves.append({
            'position': local_to_world(mouth_right_x, mouth_y, pen_down=False),
            'description': 'Lift pen after mouth',
            'pen_down': False
        })
        
        # 5. Final lift move - go higher to clearly signal completion
        moves.append({
            'position': local_to_world(center_x, center_y, pen_down=False),
            'description': 'Final lift to center (smiley face complete)',
            'pen_down': False
        })
        
        logger.info(f"🎨 Generated {len(moves)} moves for {square_size_cm}x{square_size_cm}cm smiley face")
        for i, move in enumerate(moves):
            logger.info(f"🎨   {i+1}. {move['description']}: {move['position'].round(3)}")
        
        return moves
    
    async def _execute_drawing_moves_sequence(self, arm: str, drawing_moves, wait_for_arrival: bool = True):
        """Execute the drawing moves with smooth animation and proper timing.
        
        Args:
            arm: Which arm to use ('left' or 'right')
            drawing_moves: List of moves to execute
            wait_for_arrival: If True, wait for arm to reach each position before continuing.
                            If False, move target continuously for smooth drawing.
        """
        arm_state = self.left_arm if arm == "left" else self.right_arm
        
        logger.info(f"🎨 Starting drawing sequence for {arm} arm...")
        self.drawing_in_progress = True
        
        try:
            # Execute each drawing move
            for i, move in enumerate(drawing_moves):
                logger.info(f"🎨 Move {i+1}/{len(drawing_moves)}: {move['description']}")
                target_pos = move['position']
                
                # Get current goal position as starting point
                if arm_state.goal_position is not None:
                    start_pos = arm_state.goal_position.copy()
                else:
                    start_pos = self.robot_interface.get_current_end_effector_position(arm) if self.robot_interface else target_pos
                
                # Calculate distance and movement time at 2cm/s
                distance = np.linalg.norm(target_pos - start_pos)
                speed = 0.02  # 2 cm/s
                total_duration = distance / speed if speed > 0 else 1.0
                
                logger.info(f"🎨 Distance: {distance:.3f}m, Duration: {total_duration:.1f}s")
                
                # Smooth interpolation
                control_frequency = 50.0  # 50Hz updates
                num_steps = max(10, int(total_duration * control_frequency))
                dt = total_duration / num_steps
                
                # Animate the goal position smoothly
                for step in range(num_steps + 1):
                    t_linear = step / num_steps  # 0 to 1
                    t_smooth = 0.5 * (1 - np.cos(np.pi * t_linear))  # S-curve
                    current_goal = start_pos + t_smooth * (target_pos - start_pos)
                    
                    # Update goal position
                    arm_state.goal_position = current_goal.copy()
                    arm_state.target_position = current_goal.copy()
                    arm_state.mode = ControlMode.POSITION_CONTROL
                    
                    if step < num_steps:
                        await asyncio.sleep(dt)
                
                logger.info(f"🎨 Reached: {target_pos.round(3)}")
                
                # Wait for robot arm to reach the position (only if wait_for_arrival is True)
                if wait_for_arrival and self.robot_interface and i < len(drawing_moves) - 1:
                    logger.info(f"🎨 Waiting for arm to reach position...")
                    timeout = 10.0
                    start_time = time.time()
                    
                    while time.time() - start_time < timeout:
                        current_robot_pos = self.robot_interface.get_current_end_effector_position(arm)
                        distance_to_goal = np.linalg.norm(current_robot_pos - target_pos)
                        
                        if distance_to_goal <= 0.005:  # 0.5cm
                            logger.info(f"🎨 Arm reached! Distance: {distance_to_goal*1000:.1f}mm")
                            break
                        
                        await asyncio.sleep(0.1)
                    else:
                        logger.warning(f"🎨 Timeout waiting for arm")
                elif not wait_for_arrival:
                    logger.debug(f"🎨 Continuous mode: not waiting for arm to reach position")
            
            logger.info("🎨 Drawing sequence complete!")
            
        except Exception as e:
            logger.error(f"🎨 Error during drawing sequence: {e}")
        finally:
            self.drawing_in_progress = False
        
    async def _move_goal_marker_sequence(self, arm: str, saved_positions):
        """Move the goal marker smoothly at 2cm/s and wait for arm to reach each position."""
        arm_state = self.left_arm if arm == "left" else self.right_arm
        
        logger.info(f"🎨 Starting smooth goal marker movement for {arm} arm...")
        
        try:
            # Move goal marker to each saved position smoothly
            for i, target_pos in enumerate(saved_positions):
                logger.info(f"🎨 Moving to point {i+1}/4: {target_pos.round(3)}")
                
                # Get current goal position as starting point
                if arm_state.goal_position is not None:
                    start_pos = arm_state.goal_position.copy()
                else:
                    # If no goal position set yet, use current robot position
                    start_pos = self.robot_interface.get_current_end_effector_position(arm) if self.robot_interface else target_pos
                
                # Calculate distance and movement time at 2cm/s
                distance = np.linalg.norm(target_pos - start_pos)
                speed = 0.05  # 5 cm/s
                total_duration = distance / speed if speed > 0 else 1.0
                
                logger.info(f"🎨 Distance: {distance:.3f}m, Duration: {total_duration:.1f}s at {speed:.3f}m/s")
                
                # Smooth interpolation
                control_frequency = 50.0  # 50Hz updates
                num_steps = max(10, int(total_duration * control_frequency))
                dt = total_duration / num_steps
                
                # Animate the goal position smoothly
                for step in range(num_steps + 1):
                    # Smooth interpolation with ease-in-out
                    t_linear = step / num_steps  # 0 to 1
                    t_smooth = 0.5 * (1 - np.cos(np.pi * t_linear))  # S-curve
                    current_goal = start_pos + t_smooth * (target_pos - start_pos)
                    
                    # Update goal position
                    arm_state.goal_position = current_goal.copy()
                    arm_state.target_position = current_goal.copy()
                    arm_state.mode = ControlMode.POSITION_CONTROL
                    
                    if step < num_steps:  # Don't wait after last step
                        await asyncio.sleep(dt)
                
                logger.info(f"🎨 Goal reached: {target_pos.round(3)}")
                
                # Wait for robot arm to actually reach the position (within 0.5cm)
                if self.robot_interface and i < len(saved_positions) - 1:  # Don't wait after last move
                    logger.info(f"🎨 Waiting for arm to reach position within 0.5cm...")
                    timeout = 10.0  # 10 second timeout
                    start_time = time.time()
                    
                    while time.time() - start_time < timeout:
                        current_robot_pos = self.robot_interface.get_current_end_effector_position(arm)
                        distance_to_goal = np.linalg.norm(current_robot_pos - target_pos)
                        
                        if distance_to_goal <= 0.005:  # 0.5cm
                            logger.info(f"🎨 Arm reached position! Distance: {distance_to_goal*1000:.1f}mm")
                            break
                        
                        await asyncio.sleep(0.1)  # Check every 100ms
                    else:
                        logger.warning(f"🎨 Timeout waiting for arm to reach position")
            
            logger.info("🎨 Smooth goal marker sequence complete!")
            
        except Exception as e:
            logger.error(f"🎨 Error during smooth goal marker movement: {e}")
        
    async def _initialize_drawing_mode(self, arm: str):
        """Initialize drawing mode like VR grip activation."""
        # Activate position control and reset target to current position (like VR grip press)
        reset_goal = ControlGoal(
            arm=arm,
            mode=ControlMode.POSITION_CONTROL,
            target_position=None,  # Special signal
            metadata={
                "source": "drawing_init",
                "reset_target_to_current": True
            }
        )
        await self._execute_goal(reset_goal)
        logger.info(f"🎨 {arm.upper()} arm: Drawing mode initialized")
        
    async def _end_drawing_mode(self, arm: str):
        """End drawing mode like VR grip release."""
        idle_goal = ControlGoal(
            arm=arm,
            mode=ControlMode.IDLE,
            metadata={"source": "drawing_end"}
        )
        await self._execute_goal(idle_goal)
        logger.info(f"🎨 {arm.upper()} arm: Drawing mode ended")
        
    async def _move_to_position_smooth(self, arm: str, target_absolute_pos: np.ndarray, speed: float):
        """Move to position using direct smooth trajectory interpolation (bypasses all VR systems)."""
        arm_state = self.left_arm if arm == "left" else self.right_arm
        
        # Get starting position - use current target if available (for chained movements), otherwise robot position
        if arm_state.target_position is not None:
            start_pos = arm_state.target_position.copy()
            logger.info(f"🎬 Using current target position as start: {start_pos.round(3)}")
        elif self.robot_interface:
            start_pos = self.robot_interface.get_current_end_effector_position(arm)
            logger.info(f"🎬 Using robot position as start: {start_pos.round(3)}")
        else:
            logger.error("🎬 No robot interface available for smooth movement")
            return
            
        # Calculate total distance and movement time
        total_distance = np.linalg.norm(target_absolute_pos - start_pos)
        total_duration = total_distance / speed if speed > 0 else 1.0
        
        logger.info(f"🎬 Smooth trajectory from {start_pos.round(3)} to {target_absolute_pos.round(3)}")
        logger.info(f"🎬 Distance: {total_distance:.3f}m, Duration: {total_duration:.1f}s, Speed: {speed:.3f}m/s")
        
        # Create smooth interpolation that matches robot control frequency
        control_frequency = 1.0 / self.config.send_interval  # Hz (usually 50Hz now)
        num_steps = max(10, int(total_duration * control_frequency))  # Steps at robot frequency
        dt = total_duration / num_steps
        
        logger.info(f"🎬 Control frequency: {control_frequency:.1f}Hz, Steps: {num_steps}, dt: {dt:.3f}s")
        
        # Activate position control mode and ensure clean state
        arm_state.mode = ControlMode.POSITION_CONTROL
        arm_state.origin_position = None  # Clear origin to prevent VR system interference
        
        for i in range(num_steps + 1):
            # Smooth interpolation with ease-in-out for natural motion
            t_linear = i / num_steps  # 0 to 1
            # Apply smooth curve: ease-in-out using cosine interpolation
            t_smooth = 0.5 * (1 - np.cos(np.pi * t_linear))  # S-curve acceleration/deceleration
            current_target = start_pos + t_smooth * (target_absolute_pos - start_pos)
            
            logger.debug(f"🎬 Step {i+1}/{num_steps+1}: {current_target.round(3)} (t={t_smooth:.3f})")
            
            # Set the target position directly - this bypasses all VR relative positioning
            arm_state.target_position = current_target.copy()
            arm_state.goal_position = current_target.copy()
            
            # Wait exactly one robot control cycle
            if i < num_steps:  # Don't wait after the last step
                await asyncio.sleep(dt)
        
        logger.info(f"🎬 Smooth trajectory complete! Final position: {target_absolute_pos.round(3)}")
    
    async def _move_to_position_like_vr(self, arm: str, target_absolute_pos: np.ndarray, speed: float):
        """Move to position using the same method as VR controllers (origin-based)."""
        arm_state = self.left_arm if arm == "left" else self.right_arm
        
        # Ensure arm is in position control mode before each move
        if arm_state.mode != ControlMode.POSITION_CONTROL:
            logger.warning(f"🎨 {arm.upper()} arm not in position control mode, reactivating...")
            await self._initialize_drawing_mode(arm)
        
        if arm_state.origin_position is None:
            logger.error("No origin position set for drawing, reinitializing...")
            await self._initialize_drawing_mode(arm)
            return
            
        # Calculate relative position from origin (like VR does)
        relative_delta = target_absolute_pos - arm_state.origin_position
        
        logger.info(f"🎨 Origin: {arm_state.origin_position.round(3)}")
        logger.info(f"🎨 Target absolute: {target_absolute_pos.round(3)}")
        logger.info(f"🎨 Relative delta: {relative_delta.round(3)}")
        
        # Send position control goal like VR does
        goal = ControlGoal(
            arm=arm,
            mode=ControlMode.POSITION_CONTROL,  # Explicitly set mode each time
            target_position=relative_delta,  # Relative position delta
            metadata={
                "source": "drawing_execution",
                "relative_position": True,
                "origin_position": arm_state.origin_position.copy()
            }
        )
        
        # Execute the goal
        await self._execute_goal(goal)
        
        # Verify the goal was applied
        logger.info(f"🎨 Arm mode after goal: {arm_state.mode}")
        logger.info(f"🎨 Target position after goal: {arm_state.target_position.round(3) if arm_state.target_position is not None else 'None'}")
        
        # Calculate movement time and wait
        distance = np.linalg.norm(relative_delta)
        duration = distance / speed if speed > 0 else 1.0
        if duration < 0.5:  # Minimum wait time
            duration = 0.5
        
        logger.info(f"🎨 Waiting {duration:.1f}s for {distance:.3f}m movement")
        await asyncio.sleep(duration)
    
    async def _execute_smooth_test_move(self, goal: ControlGoal):
        """Execute a smooth animated test move from current position."""
        logger.info(f"🧪 Starting smooth test move for {goal.arm} arm...")
        
        try:
            # Get current robot position as starting point
            if self.robot_interface:
                current_pos = self.robot_interface.get_current_end_effector_position(goal.arm)
            else:
                logger.error("🧪 No robot interface available")
                return
            
            # Calculate target absolute position from current position (simple addition)
            target_absolute_pos = current_pos + goal.target_position
            
            logger.info(f"🧪 Moving from current: {current_pos.round(3)}")
            logger.info(f"🧪 Moving to target: {target_absolute_pos.round(3)}")
            logger.info(f"🧪 Delta: {goal.target_position.round(3)}")
            
            # Execute direct smooth movement (no origin-based system)
            test_speed = 0.02  # 2 cm/s - faster than drawing for testing
            await self._move_to_position_smooth(goal.arm, target_absolute_pos, test_speed)
            
            logger.info("🧪 Smooth test move complete!")
            
        except Exception as e:
            logger.error(f"🧪 Error during smooth test move: {e}")
        finally:
            # Clear the test move flag
            self._test_move_active = False
            
            # Don't end position control mode - leave it active for multiple test moves
            logger.info(f"🧪 {goal.arm.upper()} arm: Staying in position control for additional moves")
    
    async def _initialize_test_move_mode(self, arm: str):
        """Initialize test move mode like drawing mode."""
        # Activate position control and reset target to current position
        reset_goal = ControlGoal(
            arm=arm,
            mode=ControlMode.POSITION_CONTROL,
            target_position=None,  # Special signal
            metadata={
                "source": "test_move_init",
                "reset_target_to_current": True
            }
        )
        await self._execute_goal(reset_goal)
        logger.info(f"🧪 {arm.upper()} arm: Test move mode initialized")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        } 
