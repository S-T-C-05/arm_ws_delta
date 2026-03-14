"""Signal conversion utilities for the robotic arm.

Provides helpers for converting between raw hardware signals (encoder counts,
ADC values) and physical / ROS units (radians).

All conversion constants can be overridden at construction time to match the
actual hardware configuration.
"""

import math


class SignalUtils:
    """Utility class for signal conversions between hardware and ROS units.

    Args:
        encoder_cpr:       Encoder counts per revolution (default 4096).
        adc_max:           Maximum raw ADC value (default 4095 for 12-bit ADC).
        pot_min_rad:       Potentiometer angle at raw=0 (radians, default 0.0).
        pot_max_rad:       Potentiometer angle at raw=adc_max (radians, default pi).
        gear_ratio:        Motor-to-joint gear ratio (default 1.0).
    """

    def __init__(
        self,
        encoder_cpr: int = 4096,
        adc_max: int = 4095,
        pot_min_rad: float = 0.0,
        pot_max_rad: float = math.pi,
        gear_ratio: float = 1.0,
    ) -> None:
        self.encoder_cpr = encoder_cpr
        self.adc_max = adc_max
        self.pot_min_rad = pot_min_rad
        self.pot_max_rad = pot_max_rad
        self.gear_ratio = gear_ratio

    # ------------------------------------------------------------------
    # Encoder conversions
    # ------------------------------------------------------------------

    def encoder_to_radians(self, counts: int) -> float:
        """Convert encoder counts to joint angle in radians.

        Args:
            counts: Raw encoder count (cumulative, signed).

        Returns:
            Joint angle in radians.
        """
        counts_per_joint_rev = self.encoder_cpr * self.gear_ratio
        return (counts / counts_per_joint_rev) * 2.0 * math.pi

    def radians_to_encoder(self, radians: float) -> int:
        """Convert a joint angle in radians to the equivalent encoder count.

        Args:
            radians: Desired joint angle in radians.

        Returns:
            Target encoder count (integer).
        """
        counts_per_joint_rev = self.encoder_cpr * self.gear_ratio
        return int(round((radians / (2.0 * math.pi)) * counts_per_joint_rev))

    def velocity_rad_per_s_to_counts_per_s(self, vel_rad: float) -> int:
        """Convert angular velocity (rad/s) to encoder counts per second.

        Args:
            vel_rad: Velocity in radians per second.

        Returns:
            Velocity in encoder counts per second (integer).
        """
        counts_per_joint_rev = self.encoder_cpr * self.gear_ratio
        return int(round((vel_rad / (2.0 * math.pi)) * counts_per_joint_rev))

    # ------------------------------------------------------------------
    # Potentiometer / ADC conversions
    # ------------------------------------------------------------------

    def adc_to_radians(self, raw: int) -> float:
        """Convert a raw ADC reading to joint angle in radians.

        Uses linear interpolation between pot_min_rad and pot_max_rad.

        Args:
            raw: Raw ADC value in [0, adc_max].

        Returns:
            Joint angle in radians.
        """
        raw_clamped = max(0, min(self.adc_max, raw))
        t = raw_clamped / self.adc_max
        return self.pot_min_rad + t * (self.pot_max_rad - self.pot_min_rad)

    def radians_to_adc(self, radians: float) -> int:
        """Convert a joint angle in radians to the expected ADC raw value.

        Useful for calibration and simulation.

        Args:
            radians: Joint angle in radians.

        Returns:
            Estimated raw ADC value in [0, adc_max].
        """
        span = self.pot_max_rad - self.pot_min_rad
        if span == 0.0:
            return 0
        t = (radians - self.pot_min_rad) / span
        t = max(0.0, min(1.0, t))
        return int(round(t * self.adc_max))

    # ------------------------------------------------------------------
    # Wrist differential conversions
    # ------------------------------------------------------------------

    @staticmethod
    def differential_to_pitch_roll(enc_m1: int, enc_m2: int,
                                   cpr: int = 4096,
                                   gear_ratio: float = 1.0) -> tuple:
        """Convert two differential-drive encoder counts to pitch and roll angles.

        The wrist uses two motors as a differential:
          - pitch = (M1 + M2) / 2
          - roll  = (M1 - M2) / 2

        Args:
            enc_m1:     Encoder count for motor 1.
            enc_m2:     Encoder count for motor 2.
            cpr:        Counts per revolution.
            gear_ratio: Motor-to-joint gear ratio.

        Returns:
            (pitch_rad, roll_rad) tuple.
        """
        k = (2.0 * math.pi) / (cpr * gear_ratio)
        pitch_rad = k * (enc_m1 + enc_m2) / 2.0
        roll_rad = k * (enc_m1 - enc_m2) / 2.0
        return (pitch_rad, roll_rad)

    @staticmethod
    def pitch_roll_to_differential(pitch_rad: float, roll_rad: float,
                                   cpr: int = 4096,
                                   gear_ratio: float = 1.0) -> tuple:
        """Convert desired pitch and roll angles to differential motor counts.

        Inverse of differential_to_pitch_roll.

        Args:
            pitch_rad:  Desired pitch angle in radians.
            roll_rad:   Desired roll angle in radians.
            cpr:        Counts per revolution.
            gear_ratio: Motor-to-joint gear ratio.

        Returns:
            (enc_m1, enc_m2) encoder count targets as integers.
        """
        k = (cpr * gear_ratio) / (2.0 * math.pi)
        enc_m1 = int(round(k * (pitch_rad + roll_rad)))
        enc_m2 = int(round(k * (pitch_rad - roll_rad)))
        return (enc_m1, enc_m2)
