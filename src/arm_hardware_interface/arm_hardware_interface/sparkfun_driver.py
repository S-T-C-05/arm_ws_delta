"""SparkFun serial motor driver interface.

Manages the base rotation motor connected via a SparkFun Serial Controlled
Motor Driver (SCMD).  The SCMD uses a simple serial ASCII protocol to set
PWM duty-cycle and direction for a single motor channel.

Protocol reference (SparkFun SCMD ASCII mode):
  - Drive command:  "M <channel> <speed -255..255>\\n"
  - Stop:           "M <channel> 0\\n"
  - (Encoder read delegated to STM32 firmware over a separate connection)
"""

import serial
from typing import Dict, List, Optional

from .hardware_interface_base import HardwareInterfaceBase


CHANNEL_BASE = 0


class SparkFunDriver(HardwareInterfaceBase):
    """Driver for the SparkFun Serial Controlled Motor Driver (base motor).

    Args:
        port:     Serial port device path (e.g. '/dev/ttyUSB0').
        baudrate: Serial baud rate (default 115200).
        timeout:  Read timeout in seconds (default 0.1).
    """

    def __init__(self, port: str = '/dev/ttyUSB0',
                 baudrate: int = 115200, timeout: float = 0.1) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._encoder_count: int = 0

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the serial connection to the SparkFun SCMD."""
        try:
            self._serial = serial.Serial(
                self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            return self._serial.is_open
        except serial.SerialException:
            self._serial = None
            return False

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            try:
                self.stop()
                self._serial.close()
            except serial.SerialException:
                pass

    def is_connected(self) -> bool:
        """Return True if the serial port is open."""
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # Motor commands
    # ------------------------------------------------------------------

    def set_velocity(self, channel: int, velocity: int) -> bool:
        """Send a signed PWM velocity command (-255 to 255).

        Args:
            channel:  Motor channel (use CHANNEL_BASE = 0 for base motor).
            velocity: PWM duty cycle, signed (-255 stop reverse, +255 full forward).
        """
        velocity = max(-255, min(255, velocity))
        cmd = f'M {channel} {velocity}\n'
        return self._send(cmd)

    def set_position(self, channel: int, position: int, speed: int = 100) -> bool:
        """Position control is handled externally by the STM32 PID loop.

        This method is a no-op placeholder; the STM32 serial bridge
        (arm_firmware) reads encoder feedback and generates velocity commands.
        """
        return False

    def stop(self, channel: Optional[int] = None) -> bool:
        """Stop the base motor."""
        ch = CHANNEL_BASE if channel is None else channel
        return self.set_velocity(ch, 0)

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    def read_encoders(self) -> Dict[int, int]:
        """Return the last known encoder count received from the STM32 bridge.

        Encoder data is injected externally by arm_firmware via
        update_encoder_count().

        Returns:
            {CHANNEL_BASE: count}
        """
        return {CHANNEL_BASE: self._encoder_count}

    def update_encoder_count(self, count: int) -> None:
        """Update the cached encoder count (called by arm_firmware bridge).

        Args:
            count: Latest encoder count from STM32.
        """
        self._encoder_count = count

    # ------------------------------------------------------------------
    # ROS joint mapping
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> List[str]:
        """Return the ROS joint names driven by this controller."""
        return ['bracket_joint']

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send(self, command: str) -> bool:
        """Encode and write a command string to the serial port."""
        if not self.is_connected():
            return False
        try:
            self._serial.write(command.encode('ascii'))
            return True
        except serial.SerialException:
            return False
