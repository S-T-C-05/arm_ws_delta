"""Sensor reader for encoders and potentiometers connected via STM32.

The STM32 microcontroller reads:
  - 3 quadrature encoders (base, wrist M1, wrist M2)
  - 2 potentiometers (forearm actuator, humerus actuator)

It streams telemetry over a serial port using a simple CSV line format:

    "ENC:<enc_base>,<enc_wrist1>,<enc_wrist2>;POT:<pot0_raw>,<pot1_raw>\\n"

This reader node parses that stream and exposes the latest readings through
properties and ROS topic callbacks.
"""

from typing import Optional

import serial


class SensorReader:
    """Reads encoder and potentiometer data from the STM32 serial stream.

    Args:
        port:     Serial device path (e.g. '/dev/ttyACM1').
        baudrate: Baud rate (default 115200).
        timeout:  Read timeout in seconds (default 0.05).
    """

    # Sentinel value when no data has been received yet
    NO_DATA = None

    def __init__(self, port: str = '/dev/ttyACM1',
                 baudrate: int = 115200, timeout: float = 0.05) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial: Optional[serial.Serial] = None

        # Latest readings
        self.enc_base: int = 0
        self.enc_wrist1: int = 0
        self.enc_wrist2: int = 0
        self.pot_forearm_raw: int = 0
        self.pot_humerus_raw: int = 0

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open serial connection to the STM32."""
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
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass

    def is_connected(self) -> bool:
        """Return True if the serial port is open."""
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # Data acquisition
    # ------------------------------------------------------------------

    def read_once(self) -> bool:
        """Read and parse one telemetry line from the STM32.

        Expected format::

            "ENC:<enc_base>,<enc_wrist1>,<enc_wrist2>;POT:<pot0_raw>,<pot1_raw>\\n"

        Returns:
            True if a valid line was received and parsed.
        """
        if not self.is_connected():
            return False
        try:
            raw = self._serial.readline()
            line = raw.decode('ascii', errors='ignore').strip()
            return self._parse_line(line)
        except serial.SerialException:
            return False

    def _parse_line(self, line: str) -> bool:
        """Parse one telemetry line and update internal state.

        Args:
            line: Decoded telemetry string from STM32.

        Returns:
            True if parsing succeeded.
        """
        # Expected: "ENC:0,0,0;POT:2048,2048"
        try:
            enc_part, pot_part = line.split(';')
            enc_vals = enc_part.lstrip('ENC:').split(',')
            pot_vals = pot_part.lstrip('POT:').split(',')

            self.enc_base = int(enc_vals[0])
            self.enc_wrist1 = int(enc_vals[1])
            self.enc_wrist2 = int(enc_vals[2])
            self.pot_humerus_raw = int(pot_vals[0])
            self.pot_forearm_raw = int(pot_vals[1])
            return True
        except (ValueError, IndexError):
            return False

    # ------------------------------------------------------------------
    # Convenience properties (physical units via SignalUtils)
    # ------------------------------------------------------------------

    @property
    def all_encoders(self) -> dict:
        """Return all encoder counts as a dict."""
        return {
            'enc_base': self.enc_base,
            'enc_wrist1': self.enc_wrist1,
            'enc_wrist2': self.enc_wrist2,
        }

    @property
    def all_potentiometers(self) -> dict:
        """Return all raw ADC potentiometer values as a dict."""
        return {
            'pot_humerus_raw': self.pot_humerus_raw,
            'pot_forearm_raw': self.pot_forearm_raw,
        }
