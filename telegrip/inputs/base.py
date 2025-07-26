"""
Base classes and data structures for input providers.
"""

import asyncio
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Literal, Optional

import numpy as np


class ControlMode(Enum):
    """Control modes for the teleoperation system."""

    POSITION_CONTROL = "position"
    IDLE = "idle"


@dataclass
class ControlGoal:
    """High-level control goal message sent from input providers."""

    arm: Literal["left", "right"]
    mode: Optional[ControlMode] = None  # Control mode (None = no mode change)
    target_transform: Optional[np.ndarray] = None  # 4x4 transform matrix
    gripper_closed: Optional[bool] = None  # Gripper state (None = no change)

    # Additional data for debugging/monitoring
    metadata: Optional[Dict[str, Any]] = None


class BaseInputProvider(ABC):
    """Abstract base class for input providers."""

    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False

    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass

    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass

    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            # Handle queue full or other errors
            pass
