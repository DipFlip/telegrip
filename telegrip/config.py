"""
Configuration module for the unified teleoperation system.
Loads configuration from config.yaml file with fallback to default values.
"""

import os
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path
import logging
from .utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# Default configuration values (fallback if YAML file doesn't exist)
DEFAULT_CONFIG = {
    "network": {
        "https_port": 8443,
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
    },
    "ssl": {
        "certfile": "cert.pem",
        "keyfile": "key.pem"
    },
    "robot": {
        "type": "so100",  # "so100" for 1-2 arms, "xlerobot" for full xlerobot with wheels
        "left_arm": {
            "name": "Left Arm",
            "port": "/dev/ttyACM0",
            "enabled": True
        },
        "right_arm": {
            "name": "Right Arm",
            "port": "/dev/ttyACM1",
            "enabled": True
        },
        "vr_to_robot_scale": 1.0,
        "send_interval": 0.05,
    },
    "control": {
        "keyboard": {
            "enabled": True,
            "pos_step": 0.01,
            "angle_step": 5.0,
            "gripper_step": 10.0
        },
        "vr": {
            "enabled": True
        },
        "pybullet": {
            "enabled": True
        }
    },
    "paths": {
        "urdf_path": "URDF/SO100/so100.urdf"
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0
    },
    "ik": {
        "use_reference_poses": True,
        "reference_poses_file": "reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file with fallback to defaults."""
    config = DEFAULT_CONFIG.copy()
    
    # Try to load from project root first (package installation directory)
    package_config_path = get_absolute_path(config_path)
    
    # Check if config exists in package directory
    if package_config_path.exists():
        config_file_to_use = package_config_path
        logger.info(f"Loading config from package directory: {config_file_to_use}")
    # Fallback to current working directory (for user-provided configs)
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        logger.info(f"Loading config from current directory: {config_file_to_use}")
    else:
        logger.info(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # Deep merge yaml config into default config
                _deep_merge(config, yaml_config)
    except Exception as e:
        logger.warning(f"Could not load config from {config_file_to_use}: {e}")
        logger.info("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """Save configuration to YAML file in project root."""
    # Always save to project root directory
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Error saving config to {abs_config_path}: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """Deep merge update dict into base dict."""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value

# Load configuration
_config_data = load_config()

# Extract values for backward compatibility
HTTPS_PORT = _config_data["network"]["https_port"]
WEBSOCKET_PORT = _config_data["network"]["websocket_port"]
HOST_IP = _config_data["network"]["host_ip"]

CERTFILE = _config_data["ssl"]["certfile"]
KEYFILE = _config_data["ssl"]["keyfile"]

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"]
SEND_INTERVAL = _config_data["robot"]["send_interval"]
ROBOT_TYPE = _config_data["robot"].get("type", "so100")  # "so100" or "xlerobot"

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]
GRIPPER_STEP = _config_data["control"]["keyboard"]["gripper_step"]

URDF_PATH = _config_data["paths"]["urdf_path"]

GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# --- Base (Wheel) Configuration ---
# Wheel motors on right arm bus (motors 7, 8, 9)
WHEEL_MOTOR_NAMES = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]

# Wheel physical parameters
WHEEL_RADIUS = 0.05  # meters (5cm)
WHEEL_BASE_RADIUS = 0.125  # distance from robot center to each wheel (meters)

# Base speed settings
BASE_SPEED_LEVELS = [
    {"linear": 0.1, "angular": 30},   # slow: 0.1 m/s, 30 deg/s
    {"linear": 0.2, "angular": 60},   # medium: 0.2 m/s, 60 deg/s
    {"linear": 0.3, "angular": 90},   # fast: 0.3 m/s, 90 deg/s
    {"linear": 0.5, "angular": 120},  # very fast: 0.5 m/s, 120 deg/s
]
DEFAULT_BASE_SPEED_INDEX = 1  # Start at medium speed

# IK Configuration
USE_REFERENCE_POSES = _config_data["ik"]["use_reference_poses"]
REFERENCE_POSES_FILE = _config_data["ik"]["reference_poses_file"]
IK_POSITION_ERROR_THRESHOLD = _config_data["ik"]["position_error_threshold"]
IK_HYSTERESIS_THRESHOLD = _config_data["ik"]["hysteresis_threshold"]
IK_MOVEMENT_PENALTY_WEIGHT = _config_data["ik"]["movement_penalty_weight"]

# --- Joint Configuration ---
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 3  # Use only first 3 joints for IK (Rotation, Pitch, Elbow)
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# Motor configuration for SO100
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# URDF joint name mapping
URDF_TO_INTERNAL_NAME_MAP = {
    "1": "shoulder_pan",
    "2": "shoulder_lift",
    "3": "elbow_flex",
    "4": "wrist_flex",
    "5": "wrist_roll",
    "6": "gripper",
}

# --- PyBullet Configuration ---
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip"

# --- Keyboard Control ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

# --- Device Ports ---
DEFAULT_FOLLOWER_PORTS = {
    "left": _config_data["robot"]["left_arm"]["port"],
    "right": _config_data["robot"]["right_arm"]["port"]
}

@dataclass
class TelegripConfig:
    """Main configuration class for the teleoperation system."""
    
    # Network settings
    https_port: int = HTTPS_PORT
    websocket_port: int = WEBSOCKET_PORT
    host_ip: str = HOST_IP
    
    # SSL settings
    certfile: str = CERTFILE
    keyfile: str = KEYFILE
    
    # Robot settings
    robot_type: str = ROBOT_TYPE  # "so100" for 1-2 arms, "xlerobot" for full xlerobot with wheels
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL

    # Device ports
    follower_ports: Dict[str, str] = None
    
    # Control flags
    enable_pybullet: bool = True
    enable_pybullet_gui: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    enable_keyboard: bool = True
    autoconnect: bool = False
    log_level: str = "warning"
    
    # Paths
    urdf_path: str = URDF_PATH
    webapp_dir: str = "webapp"
    
    # IK settings
    use_reference_poses: bool = USE_REFERENCE_POSES
    reference_poses_file: str = REFERENCE_POSES_FILE
    ik_position_error_threshold: float = IK_POSITION_ERROR_THRESHOLD
    ik_hysteresis_threshold: float = IK_HYSTERESIS_THRESHOLD
    ik_movement_penalty_weight: float = IK_MOVEMENT_PENALTY_WEIGHT
    
    # Gripper settings
    gripper_open_angle: float = GRIPPER_OPEN_ANGLE
    gripper_closed_angle: float = GRIPPER_CLOSED_ANGLE
    
    # Keyboard control
    pos_step: float = POS_STEP
    angle_step: float = ANGLE_STEP
    gripper_step: float = GRIPPER_STEP
    
    def __post_init__(self):
        # Initialize follower_ports if not set
        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],
                "right": _config_data["robot"]["right_arm"]["port"]
            }
        
        # Ensure ports are not None
        if self.follower_ports["left"] is None:
            self.follower_ports["left"] = "/dev/ttyACM0"
        if self.follower_ports["right"] is None:
            self.follower_ports["right"] = "/dev/ttyACM1"
    
    @property
    def ssl_files_exist(self) -> bool:
        """Check if SSL certificate files exist."""
        cert_path = get_absolute_path(self.certfile)
        key_path = get_absolute_path(self.keyfile)
        return cert_path.exists() and key_path.exists()
    
    def ensure_ssl_certificates(self) -> bool:
        """Ensure SSL certificates exist, generating them if necessary."""
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
    
    @property
    def urdf_exists(self) -> bool:
        """Check if URDF file exists."""
        urdf_path = get_absolute_path(self.urdf_path)
        return urdf_path.exists()
    
    @property
    def webapp_exists(self) -> bool:
        """Check if webapp directory exists."""
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_path(self) -> str:
        """Get absolute path to URDF file."""
        return str(get_absolute_path(self.urdf_path))
    
    def get_absolute_reference_poses_path(self) -> str:
        """Get absolute path to reference poses file."""
        return str(get_absolute_path(self.reference_poses_file))
    
    def get_absolute_ssl_paths(self) -> tuple:
        """Get absolute paths to SSL certificate files."""
        cert_path = str(get_absolute_path(self.certfile))
        key_path = str(get_absolute_path(self.keyfile))
        return cert_path, key_path

def get_config_data():
    """Get the current configuration data."""
    return _config_data.copy()

def update_config_data(new_config: dict):
    """Update the global configuration data."""
    global _config_data
    _config_data = new_config
    
    # Save to file
    save_config(_config_data)

# Global configuration instance
config = TelegripConfig() 
