"""Abstract base class for all arm hardware interfaces."""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional


class HardwareInterfaceBase(ABC):
    """Abstract base class that all hardware drivers must implement.

    Provides a uniform API for commanding motors and reading sensor feedback,
    enabling transparent switching between simulation (mock) and real hardware.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Open the connection to the hardware device.

        Returns:
            True if the connection was established successfully, False otherwise.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Close the connection to the hardware device gracefully."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return True if the hardware connection is active."""

    @abstractmethod
    def set_velocity(self, channel: int, velocity: int) -> bool:
        """Command a motor channel to a given velocity.

        Args:
            channel: Motor channel index (driver-specific).
            velocity: Commanded velocity (driver-specific units, e.g. encoder counts/s).

        Returns:
            True if the command was sent successfully.
        """

    @abstractmethod
    def set_position(self, channel: int, position: int, speed: int) -> bool:
        """Command a motor channel to a target position with closed-loop PID.

        Args:
            channel: Motor channel index (driver-specific).
            position: Target position in encoder counts.
            speed:    Maximum speed in encoder counts/s.

        Returns:
            True if the command was sent successfully.
        """

    @abstractmethod
    def stop(self, channel: Optional[int] = None) -> bool:
        """Stop one or all motor channels.

        Args:
            channel: Channel to stop, or None to stop all channels.

        Returns:
            True if the stop command was sent successfully.
        """

    @abstractmethod
    def read_encoders(self) -> Dict[int, int]:
        """Read encoder counts for all available motor channels.

        Returns:
            Dictionary mapping channel index to encoder count.
        """

    @property
    @abstractmethod
    def joint_names(self) -> List[str]:
        """Return the list of ROS joint names managed by this interface."""

    def __repr__(self) -> str:
        status = 'connected' if self.is_connected() else 'disconnected'
        return f'{self.__class__.__name__}({status})'
