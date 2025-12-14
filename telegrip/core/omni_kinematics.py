r"""
3-wheel omnidirectional drive kinematics for the mobile base.

Wheel layout (looking from above):
       Front (X+)
         ^
        / \
       /   \
      L     R   <- Left (motor 7) and Right (motor 9) wheels
       \   /
        \ /
         B      <- Back (motor 8) wheel

Body frame:
- X: Forward (positive = robot moves forward)
- Y: Left (positive = robot moves left / strafes left)
- Theta: Counter-clockwise rotation (positive = robot rotates CCW)
"""

import numpy as np
from typing import Dict, Tuple

from ..config import WHEEL_RADIUS, WHEEL_BASE_RADIUS


def body_to_wheel_velocities(
    x_vel: float,
    y_vel: float,
    theta_vel: float,
    wheel_radius: float = WHEEL_RADIUS,
    base_radius: float = WHEEL_BASE_RADIUS,
    max_raw: int = 3000,
) -> Dict[str, int]:
    """
    Convert desired body-frame velocities into wheel raw velocity commands.

    Parameters:
        x_vel: Linear velocity in x (m/s), positive = forward
        y_vel: Linear velocity in y (m/s), positive = left (strafe)
        theta_vel: Rotational velocity (deg/s), positive = counter-clockwise
        wheel_radius: Radius of each wheel (meters)
        base_radius: Distance from robot center to each wheel (meters)
        max_raw: Maximum allowed raw command (steps/s) per wheel

    Returns:
        Dictionary with wheel raw velocity commands:
        {"base_left_wheel": value, "base_back_wheel": value, "base_right_wheel": value}
    """
    # Convert rotational velocity from deg/s to rad/s
    theta_rad = theta_vel * (np.pi / 180.0)

    # Create body velocity vector [x, y, theta_rad]
    velocity_vector = np.array([x_vel, y_vel, theta_rad])

    # Define wheel mounting angles with -90 degree offset
    # Wheels are at 240, 0, 120 degrees (left, back, right)
    angles = np.radians(np.array([240, 0, 120]) - 90)

    # Build kinematic matrix: each row maps body velocities to wheel linear speed
    # Column 1: cos(angle) - contribution from x velocity
    # Column 2: sin(angle) - contribution from y velocity
    # Column 3: base_radius - contribution from rotation
    kinematic_matrix = np.array([
        [np.cos(a), np.sin(a), base_radius] for a in angles
    ])

    # Compute each wheel's linear speed (m/s)
    wheel_linear_speeds = kinematic_matrix.dot(velocity_vector)

    # Convert to angular speeds (rad/s)
    wheel_angular_speeds = wheel_linear_speeds / wheel_radius

    # Convert to deg/s
    wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

    # Scale down if any wheel exceeds max_raw
    steps_per_deg = 4096.0 / 360.0
    raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
    max_raw_computed = max(raw_floats)
    if max_raw_computed > max_raw:
        scale = max_raw / max_raw_computed
        wheel_degps = wheel_degps * scale

    # Convert each wheel's angular speed (deg/s) to raw integer
    wheel_raw = [_degps_to_raw(degps) for degps in wheel_degps]

    return {
        "base_left_wheel": wheel_raw[0],
        "base_back_wheel": wheel_raw[1],
        "base_right_wheel": wheel_raw[2],
    }


def _degps_to_raw(degps: float) -> int:
    """
    Convert angular velocity in deg/s to raw motor command (steps/s).

    Parameters:
        degps: Angular velocity in degrees per second

    Returns:
        Raw motor command as signed integer
    """
    steps_per_deg = 4096.0 / 360.0
    speed_in_steps = degps * steps_per_deg
    speed_int = int(round(speed_in_steps))

    # Cap to signed 16-bit range (-32768 to 32767)
    if speed_int > 0x7FFF:
        speed_int = 0x7FFF
    elif speed_int < -0x8000:
        speed_int = -0x8000

    return speed_int


def wheel_velocities_to_body(
    left_wheel_raw: int,
    back_wheel_raw: int,
    right_wheel_raw: int,
    wheel_radius: float = WHEEL_RADIUS,
    base_radius: float = WHEEL_BASE_RADIUS,
) -> Tuple[float, float, float]:
    """
    Convert wheel raw velocities back to body-frame velocities (inverse kinematics).

    Parameters:
        left_wheel_raw: Left wheel raw velocity
        back_wheel_raw: Back wheel raw velocity
        right_wheel_raw: Right wheel raw velocity
        wheel_radius: Radius of each wheel (meters)
        base_radius: Distance from robot center to each wheel (meters)

    Returns:
        Tuple of (x_vel, y_vel, theta_vel) in (m/s, m/s, deg/s)
    """
    # Convert raw to deg/s
    wheel_degps = np.array([
        _raw_to_degps(left_wheel_raw),
        _raw_to_degps(back_wheel_raw),
        _raw_to_degps(right_wheel_raw),
    ])

    # Convert to rad/s then to linear speed (m/s)
    wheel_angular_speeds = wheel_degps * (np.pi / 180.0)
    wheel_linear_speeds = wheel_angular_speeds * wheel_radius

    # Define wheel mounting angles
    angles = np.radians(np.array([240, 0, 120]) - 90)

    # Build kinematic matrix
    kinematic_matrix = np.array([
        [np.cos(a), np.sin(a), base_radius] for a in angles
    ])

    # Solve for body velocities using pseudo-inverse
    body_velocities = np.linalg.pinv(kinematic_matrix).dot(wheel_linear_speeds)

    x_vel = body_velocities[0]
    y_vel = body_velocities[1]
    theta_vel = body_velocities[2] * (180.0 / np.pi)  # Convert back to deg/s

    return x_vel, y_vel, theta_vel


def _raw_to_degps(raw_speed: int) -> float:
    """
    Convert raw motor command to angular velocity in deg/s.

    Parameters:
        raw_speed: Raw motor command (steps/s)

    Returns:
        Angular velocity in degrees per second
    """
    steps_per_deg = 4096.0 / 360.0
    return raw_speed / steps_per_deg
