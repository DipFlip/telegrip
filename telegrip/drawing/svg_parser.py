"""
SVG path parser and converter for robot drawing.
Converts SVG path data to robot drawing commands.
"""

import re
import xml.etree.ElementTree as ET
import numpy as np
from typing import List, Tuple, Dict, Optional
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


class SVGParser:
    """Parser for SVG files to extract drawing paths."""
    
    def __init__(self):
        self.current_pos = np.array([0.0, 0.0])
        
    def parse_svg_file(self, file_path: str) -> Dict:
        """Parse an SVG file and extract path information."""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Extract viewBox or width/height for coordinate system
            viewbox = self._get_viewbox(root)
            
            # Find all path elements
            paths = []
            for path in root.iter():
                if path.tag.endswith('path'):
                    path_data = path.get('d')
                    if path_data:
                        paths.append(path_data)
            
            return {
                'paths': paths,
                'viewbox': viewbox,
                'filename': Path(file_path).name
            }
            
        except Exception as e:
            logger.error(f"Error parsing SVG file {file_path}: {e}")
            return None
    
    def _get_viewbox(self, root) -> Tuple[float, float, float, float]:
        """Extract viewbox or calculate from width/height."""
        viewbox = root.get('viewBox')
        if viewbox:
            parts = viewbox.split()
            return tuple(float(p) for p in parts)
        
        # Fallback to width/height
        width = root.get('width', '100')
        height = root.get('height', '100')
        
        # Remove units (mm, px, etc.)
        width = re.sub(r'[a-zA-Z]', '', width)
        height = re.sub(r'[a-zA-Z]', '', height)
        
        return (0, 0, float(width), float(height))
    
    def parse_path_data(self, path_data: str) -> List[np.ndarray]:
        """Parse SVG path data string into a list of 2D points."""
        points = []
        self.current_pos = np.array([0.0, 0.0])
        
        # Clean up the path data and properly tokenize numbers
        # First, add spaces around command letters
        path_data = re.sub(r'([MmLlHhVvCcSsQqTtAaZz])', r' \1 ', path_data)
        # Replace commas with spaces
        path_data = re.sub(r',', ' ', path_data)
        # Add spaces around minus signs that are not at the start of numbers
        path_data = re.sub(r'(?<=\d)-', ' -', path_data)
        # Clean up multiple spaces
        path_data = re.sub(r'\s+', ' ', path_data).strip()
        
        tokens = path_data.split()
        logger.info(f"Tokenized path: {tokens[:10]}{'...' if len(tokens) > 10 else ''}")  # Show first 10 tokens
        i = 0
        
        while i < len(tokens):
            command = tokens[i]
            i += 1
            
            if command.upper() == 'M':  # Move to
                x, y = float(tokens[i]), float(tokens[i+1])
                if command.islower():  # Relative
                    self.current_pos += np.array([x, y])
                else:  # Absolute
                    self.current_pos = np.array([x, y])
                points.append(self.current_pos.copy())
                i += 2
                
                # After M command, subsequent coordinate pairs are implicit line commands
                while i + 1 < len(tokens):
                    try:
                        # Try to parse two more numbers as coordinates
                        x, y = float(tokens[i]), float(tokens[i+1])
                        if command.islower():  # Relative to original move
                            self.current_pos += np.array([x, y])
                        else:  # Absolute
                            self.current_pos = np.array([x, y])
                        points.append(self.current_pos.copy())
                        i += 2
                    except (ValueError, IndexError):
                        # Next token is not a coordinate pair, break to process as command
                        break
                
            elif command.upper() == 'L':  # Line to
                # L command can have multiple coordinate pairs
                while i + 1 < len(tokens):
                    try:
                        x, y = float(tokens[i]), float(tokens[i+1])
                        if command.islower():  # Relative
                            self.current_pos += np.array([x, y])
                        else:  # Absolute
                            self.current_pos = np.array([x, y])
                        points.append(self.current_pos.copy())
                        i += 2
                    except (ValueError, IndexError):
                        # Next token is not a coordinate pair, break to process as command
                        break
                
            elif command.upper() == 'C':  # Cubic Bézier curve
                # Cubic Bézier can have multiple sets of 6 parameters
                while i + 5 < len(tokens):
                    try:
                        # Try to parse 6 numbers for the Bézier curve
                        x1, y1 = float(tokens[i]), float(tokens[i+1])
                        x2, y2 = float(tokens[i+2]), float(tokens[i+3])
                        x, y = float(tokens[i+4]), float(tokens[i+5])
                        
                        if command.islower():  # Relative coordinates
                            cp1 = self.current_pos + np.array([x1, y1])
                            cp2 = self.current_pos + np.array([x2, y2])
                            end_point = self.current_pos + np.array([x, y])
                        else:  # Absolute coordinates
                            cp1 = np.array([x1, y1])
                            cp2 = np.array([x2, y2])
                            end_point = np.array([x, y])
                        
                        # Convert Bézier curve to line segments
                        curve_points = self._bezier_to_points(self.current_pos, cp1, cp2, end_point)
                        points.extend(curve_points[1:])  # Skip first point (already have it)
                        
                        logger.debug(f"Parsed Bézier curve from {self.current_pos.round(1)} to {end_point.round(1)} with {len(curve_points)} points")
                        self.current_pos = end_point
                        i += 6
                        
                        # Check if next token is a number (continuation) or a command
                        if i < len(tokens):
                            try:
                                float(tokens[i])
                                # Next token is a number, continue with more Bézier curves
                                continue
                            except ValueError:
                                # Next token is a command, break out of the loop
                                break
                        else:
                            # No more tokens
                            break
                            
                    except (ValueError, IndexError):
                        # Can't parse more Bézier parameters
                        break
                
            elif command.upper() == 'Z':  # Close path
                # Add line back to start if not already there
                if len(points) > 0 and not np.allclose(self.current_pos, points[0]):
                    points.append(points[0].copy())
                    self.current_pos = points[0].copy()
                # No additional tokens to consume
                
            else:
                # Check if this might be a number that got misinterpreted as a command
                try:
                    float(command)
                    logger.warning(f"Found number '{command}' where command expected - skipping token")
                    # This is likely a parsing error, skip this token
                    continue
                except ValueError:
                    logger.warning(f"Unsupported path command: {command}")
                    break
        
        logger.info(f"Parsed path with {len(points)} points")
        return points
    
    def _bezier_to_points(self, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, 
                         num_segments: int = 20) -> List[np.ndarray]:
        """Convert cubic Bézier curve to line segments."""
        points = []
        
        for i in range(num_segments + 1):
            t = i / num_segments
            
            # Cubic Bézier formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
            point = (
                (1 - t)**3 * p0 +
                3 * (1 - t)**2 * t * p1 +
                3 * (1 - t) * t**2 * p2 +
                t**3 * p3
            )
            points.append(point)
        
        return points


class SVGToRobotConverter:
    """Convert SVG coordinates to robot drawing commands."""
    
    def __init__(self, plane_info: Dict):
        self.plane_info = plane_info
        
    def convert_svg_to_drawing_moves(self, svg_data: Dict, target_size_cm: float = 2.5) -> List[Dict]:
        """Convert SVG data to robot drawing moves."""
        if not svg_data or not svg_data['paths']:
            return []
        
        # Parse the first path (for now, just handle single path SVGs)
        parser = SVGParser()
        svg_points = parser.parse_path_data(svg_data['paths'][0])
        
        if not svg_points:
            return []
        
        # Convert SVG coordinates to robot coordinates
        robot_points = self._scale_and_position_points(svg_points, svg_data['viewbox'], target_size_cm)
        
        # Generate drawing moves
        moves = self._generate_drawing_moves(robot_points)
        
        logger.info(f"Generated {len(moves)} drawing moves for SVG: {svg_data['filename']}")
        return moves
    
    def _scale_and_position_points(self, svg_points: List[np.ndarray], viewbox: Tuple, 
                                  target_size_cm: float) -> List[np.ndarray]:
        """Scale SVG points to fit on the robot's drawing plane."""
        if not svg_points:
            return []
        
        # Convert to numpy array for easier manipulation
        points = np.array(svg_points)
        
        # Get SVG bounds
        min_x, min_y = np.min(points, axis=0)
        max_x, max_y = np.max(points, axis=0)
        svg_width = max_x - min_x
        svg_height = max_y - min_y
        
        # Calculate scale to fit target size while maintaining aspect ratio
        target_size_m = target_size_cm / 100.0
        scale = min(target_size_m / svg_width, target_size_m / svg_height)
        
        # Center the drawing on the plane
        plane_center_x = self.plane_info['x_length'] / 2.0
        plane_center_y = self.plane_info['y_length'] / 2.0
        
        # Transform points
        robot_points = []
        for point in points:
            # Normalize to 0-1, scale, and center
            normalized_x = (point[0] - min_x) / svg_width if svg_width > 0 else 0.5
            normalized_y = (point[1] - min_y) / svg_height if svg_height > 0 else 0.5
            
            # Scale and center on plane
            robot_x = plane_center_x + (normalized_x - 0.5) * (svg_width * scale)
            robot_y = plane_center_y + (normalized_y - 0.5) * (svg_height * scale)
            
            robot_points.append(np.array([robot_x, robot_y]))
        
        return robot_points
    
    def _generate_drawing_moves(self, robot_points: List[np.ndarray]) -> List[Dict]:
        """Generate robot drawing moves from scaled points."""
        if not robot_points:
            return []
        
        moves = []
        pen_up_height_m = 0.02  # 2cm above plane for visibility
        
        def local_to_world(x_local: float, y_local: float, pen_down: bool = True) -> np.ndarray:
            """Convert local plane coordinates to world coordinates."""
            world_pos = (self.plane_info['origin'] + 
                        x_local * self.plane_info['x_unit'] + 
                        y_local * self.plane_info['y_unit'])
            
            if not pen_down:
                world_pos += pen_up_height_m * self.plane_info['normal']
            
            return world_pos
        
        # Start with pen up, move to first point
        first_point = robot_points[0]
        moves.append({
            'position': local_to_world(first_point[0], first_point[1], pen_down=False),
            'description': 'Move to SVG start position (pen up)',
            'pen_down': False
        })
        
        # Lower pen and start drawing
        moves.append({
            'position': local_to_world(first_point[0], first_point[1], pen_down=True),
            'description': 'Lower pen to start SVG drawing',
            'pen_down': True
        })
        
        # Draw to each subsequent point
        for i, point in enumerate(robot_points[1:], 1):
            moves.append({
                'position': local_to_world(point[0], point[1], pen_down=True),
                'description': f'Draw SVG segment {i}',
                'pen_down': True
            })
        
        # Lift pen when complete
        final_point = robot_points[-1]
        moves.append({
            'position': local_to_world(final_point[0], final_point[1], pen_down=False),
            'description': 'Lift pen (SVG drawing complete)',
            'pen_down': False
        })
        
        return moves