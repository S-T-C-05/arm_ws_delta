"""RoboClaw motor driver interface.

Manages the two wrist differential motors connected via a RoboClaw controller.
The RoboClaw exposes two independent motor channels (M1 and M2) each with an
encoder.  Internal PID is already configured on the device, so this driver
issues position/velocity commands over a USB/UART serial port using a minimal
ASCII/binary protocol.

Protocol reference (simplified ASCII mode):
  - Velocity command:  "!M1SPEED <val>\\n" / "!M2SPEED <val>\\n"
  - Position command:  "!M1POS <pos> <speed>\\n" / "!M2POS <pos> <speed>\\n"
  - Encoder read:      "?ENC\\n"  → "<enc1> <enc2>\\n"
"""

import serial
from typing import Dict, List, Optional

from .hardware_interface_base import HardwareInterfaceBase


CHANNEL_M1 = 1
CHANNEL_M2 = 2


class RoboClawDriver(HardwareInterfaceBase):
    """Driver for the RoboClaw dual-motor controller (wrist differential).

    Args:
        port:     Serial port device path (e.g. '/dev/ttyACM0').
        baudrate: Serial baud rate (default 115200).
        timeout:  Read timeout in seconds (default 0.1).
    """

    def __init__(self, port: str = '/dev/ttyACM0',
                 baudrate: int = 115200, timeout: float = 0.1) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the serial connection to the RoboClaw."""
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
        """Send a signed velocity command to one motor channel.

        Args:
            channel:  CHANNEL_M1 or CHANNEL_M2.
            velocity: Velocity in encoder counts/second (signed).
        """
        if channel == CHANNEL_M1:
            cmd = f'!M1SPEED {velocity}\n'
        elif channel == CHANNEL_M2:
            cmd = f'!M2SPEED {velocity}\n'
        else:
            return False
        return self._send(cmd)

    def set_position(self, channel: int, position: int, speed: int = 1000) -> bool:
        """Send a position command using the RoboClaw onboard PID.

        Args:
            channel:  CHANNEL_M1 or CHANNEL_M2.
            position: Target encoder count.
            speed:    Maximum speed in encoder counts/second.
        """
        if channel == CHANNEL_M1:
            cmd = f'!M1POS {position} {speed}\n'
        elif channel == CHANNEL_M2:
            cmd = f'!M2POS {position} {speed}\n'
        else:
            return False
        return self._send(cmd)

    def stop(self, channel: Optional[int] = None) -> bool:
        """Stop one or both motors."""
        ok = True
        channels = [CHANNEL_M1, CHANNEL_M2] if channel is None else [channel]
        for ch in channels:
            ok = ok and self.set_velocity(ch, 0)
        return ok

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    def read_encoders(self) -> Dict[int, int]:
        """Query encoder counts from both motor channels.

        Returns:
            {CHANNEL_M1: count_m1, CHANNEL_M2: count_m2}, or empty dict on error.
        """
        if not self.is_connected():
            return {}
        try:
            self._serial.write(b'?ENC\n')
            line = self._serial.readline().decode('ascii', errors='ignore').strip()
            parts = line.split()
            if len(parts) >= 2:
                return {
                    CHANNEL_M1: int(parts[0]),
                    CHANNEL_M2: int(parts[1]),
                }
        except (serial.SerialException, ValueError):
            pass
        return {}

    # ------------------------------------------------------------------
    # ROS joint mapping
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> List[str]:
        """Return the ROS joint names driven by this controller."""
        return ['ubracket_joint', 'endeffector_joint']

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
