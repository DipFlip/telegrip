"""
Drawing manager for robot arm drawing functionality.
Handles calibration, coordinate mapping, and drawing execution.
"""

import asyncio
import logging
import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class DrawingPoint:
    """A point in the drawing coordinate system."""
    x: float
    y: float
    z: float = 0.0  # Z is for pen up/down control
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


@dataclass
class CalibrationPoint:
    """A calibration point with VR and robot coordinates."""
    vr_position: np.ndarray
    robot_position: Optional[np.ndarray] = None
    timestamp: float = 0.0


class DrawingManager:
    """Manages drawing calibration and execution for robot arms."""
    
    def __init__(self):
        self.calibration_points: List[CalibrationPoint] = []
        self.is_calibrating = False
        self.drawing_plane = None
        self.current_drawing_arm = None
        
        # Drawing parameters
        self.pen_up_height = 0.01  # 1cm above drawing surface
        self.drawing_speed = 0.02  # m/s
        
    def start_calibration(self, arm: str):
        """Start calibration process for drawing bounds."""
        self.calibration_points.clear()
        self.is_calibrating = True
        self.current_drawing_arm = arm
        logger.info(f"🎨 Started drawing calibration for {arm} arm. Press A button on 4 corners of drawing area.")
        
    def add_calibration_point(self, vr_position: Dict, robot_position: Optional[np.ndarray] = None) -> bool:
        """Add a calibration point. Returns True if calibration is complete."""
        if not self.is_calibrating:
            logger.warning("Not in calibration mode")
            return False
            
        if len(self.calibration_points) >= 4:
            logger.warning("Already have 4 calibration points")
            return False
            
        point = CalibrationPoint(
            vr_position=np.array([vr_position['x'], vr_position['y'], vr_position['z']]),
            robot_position=robot_position,
            timestamp=asyncio.get_event_loop().time()
        )
        
        self.calibration_points.append(point)
        point_num = len(self.calibration_points)
        
        logger.info(f"🎨 Calibration point {point_num}/4 recorded: VR({point.vr_position[0]:.3f}, {point.vr_position[1]:.3f}, {point.vr_position[2]:.3f})")
        
        if len(self.calibration_points) == 4:
            self.complete_calibration()
            return True
            
        return False
        
    def complete_calibration(self):
        """Complete calibration and compute drawing plane."""
        if len(self.calibration_points) != 4:
            logger.error("Need exactly 4 calibration points")
            return False
            
        self.is_calibrating = False
        
        # Extract VR positions for plane computation
        vr_points = np.array([point.vr_position for point in self.calibration_points])
        
        # Compute drawing plane from the 4 points
        # Assume points define a rectangle: [bottom-left, bottom-right, top-right, top-left]
        self.drawing_plane = {
            'origin': vr_points[0],  # Bottom-left corner
            'x_axis': vr_points[1] - vr_points[0],  # Bottom edge
            'y_axis': vr_points[3] - vr_points[0],  # Left edge
            'normal': np.cross(vr_points[1] - vr_points[0], vr_points[3] - vr_points[0])
        }
        
        # Normalize axes for coordinate mapping
        self.drawing_plane['x_axis'] = self.drawing_plane['x_axis'] / np.linalg.norm(self.drawing_plane['x_axis'])
        self.drawing_plane['y_axis'] = self.drawing_plane['y_axis'] / np.linalg.norm(self.drawing_plane['y_axis'])
        self.drawing_plane['normal'] = self.drawing_plane['normal'] / np.linalg.norm(self.drawing_plane['normal'])
        
        # Calculate plane dimensions
        width = np.linalg.norm(vr_points[1] - vr_points[0])
        height = np.linalg.norm(vr_points[3] - vr_points[0])
        
        logger.info(f"🎨 Drawing plane calibrated: {width:.3f}m x {height:.3f}m")
        logger.info(f"🎨 Ready to draw on {self.current_drawing_arm} arm!")
        
        return True
        
    def vr_to_drawing_coordinates(self, vr_position: Dict) -> Optional[DrawingPoint]:
        """Convert VR position to 2D drawing coordinates (0-1 range)."""
        if self.drawing_plane is None:
            return None
            
        vr_pos = np.array([vr_position['x'], vr_position['y'], vr_position['z']])
        
        # Project VR position onto the drawing plane
        relative_pos = vr_pos - self.drawing_plane['origin']
        
        # Get coordinates in plane coordinate system
        x_coord = np.dot(relative_pos, self.drawing_plane['x_axis'])
        y_coord = np.dot(relative_pos, self.drawing_plane['y_axis'])
        
        # Calculate plane dimensions for normalization
        width = np.linalg.norm(self.calibration_points[1].vr_position - self.calibration_points[0].vr_position)
        height = np.linalg.norm(self.calibration_points[3].vr_position - self.calibration_points[0].vr_position)
        
        # Normalize to 0-1 range
        x_normalized = x_coord / width
        y_normalized = y_coord / height
        
        return DrawingPoint(x=x_normalized, y=y_normalized)
        
    def generate_square_path(self, size: float = 0.8) -> List[DrawingPoint]:
        """Generate a square drawing path in normalized coordinates."""
        center_x, center_y = 0.5, 0.5
        half_size = size / 2
        
        # Square corners (clockwise from bottom-left)
        corners = [
            DrawingPoint(center_x - half_size, center_y - half_size),  # Bottom-left
            DrawingPoint(center_x + half_size, center_y - half_size),  # Bottom-right
            DrawingPoint(center_x + half_size, center_y + half_size),  # Top-right
            DrawingPoint(center_x - half_size, center_y + half_size),  # Top-left
            DrawingPoint(center_x - half_size, center_y - half_size),  # Back to start
        ]
        
        return corners
        
    def drawing_to_robot_coordinates(self, drawing_point: DrawingPoint, pen_down: bool = True) -> Optional[np.ndarray]:
        """Convert normalized drawing coordinates to robot coordinates."""
        if self.drawing_plane is None:
            return None
            
        # Convert normalized coordinates back to VR coordinates
        width = np.linalg.norm(self.calibration_points[1].vr_position - self.calibration_points[0].vr_position)
        height = np.linalg.norm(self.calibration_points[3].vr_position - self.calibration_points[0].vr_position)
        
        # Scale to actual plane dimensions
        x_scaled = drawing_point.x * width
        y_scaled = drawing_point.y * height
        
        # Convert to 3D VR position on the plane
        vr_position = (self.drawing_plane['origin'] + 
                      x_scaled * self.drawing_plane['x_axis'] + 
                      y_scaled * self.drawing_plane['y_axis'])
        
        # Add pen height offset if pen is up
        if not pen_down:
            vr_position += self.pen_up_height * self.drawing_plane['normal']
            
        return vr_position
        
    def is_ready_to_draw(self) -> bool:
        """Check if drawing manager is ready to execute drawings."""
        return (self.drawing_plane is not None and 
                not self.is_calibrating and 
                self.current_drawing_arm is not None)
                
    def get_calibration_status(self) -> Dict:
        """Get current calibration status."""
        return {
            'is_calibrating': self.is_calibrating,
            'points_collected': len(self.calibration_points),
            'points_needed': 4,
            'ready_to_draw': self.is_ready_to_draw(),
            'current_arm': self.current_drawing_arm
        }