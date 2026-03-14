"""STM32 serial bridge node.

This node provides the communication layer between ROS 2 and the STM32
microcontroller firmware.

STM32 → ROS 2 (telemetry):
  The STM32 streams sensor data at a fixed rate using a simple ASCII format::

      "ENC:<enc_base>,<enc_wrist1>,<enc_wrist2>;POT:<pot_hum>,<pot_fore>\\n"

  The bridge parses these lines and publishes them as:
    /arm/encoder_counts  (sensor_msgs/JointState) – encoder positions in counts
    /arm/pot_raw         (std_msgs/Int32MultiArray) – raw ADC values

ROS 2 → STM32 (commands):
  The bridge subscribes to /arm_joint_commands (sensor_msgs/JointState) and
  forwards the desired positions to the STM32 using::

      "CMD:<base_enc>,<wrist1_enc>,<wrist2_enc>\\n"

  Position conversion (radians → encoder counts) is handled by SignalUtils
  from arm_hardware_interface.

Parameters:
  port            (str)   '/dev/ttyACM0'
  baudrate        (int)   115200
  timeout         (float) 0.05
  encoder_cpr     (int)   4096
  gear_ratio      (float) 1.0
  publish_rate_hz (float) 50.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

import serial
from typing import Optional

try:
    from arm_hardware_interface.signal_utils import SignalUtils
except ImportError:
    SignalUtils = None  # Allow standalone testing without arm_hardware_interface installed


# Joint name constants (must match the URDF)
JOINT_BASE = 'bracket_joint'
JOINT_WRIST_PITCH = 'ubracket_joint'
JOINT_WRIST_ROLL = 'endeffector_joint'


class STM32SerialBridge(Node):
    """ROS 2 node that bridges the STM32 serial telemetry stream."""

    def __init__(self) -> None:
        super().__init__('stm32_serial_bridge')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('encoder_cpr', 4096)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('publish_rate_hz', 50.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        gr = self.get_parameter('gear_ratio').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # ---- Signal utilities ----
        if SignalUtils is not None:
            self._signal = SignalUtils(encoder_cpr=cpr, gear_ratio=gr)
        else:
            self._signal = None
            self.get_logger().warn('arm_hardware_interface not found; encoder conversion disabled.')

        # ---- Serial connection ----
        self._serial: Optional[serial.Serial] = None
        try:
            self._serial = serial.Serial(port, baudrate=baud, timeout=timeout)
            self.get_logger().info(f'STM32 bridge connected on {port} @ {baud} bps')
        except serial.SerialException as exc:
            self.get_logger().warn(f'STM32 not available on {port}: {exc}')

        # ---- ROS interfaces ----
        self._pub_encoders = self.create_publisher(JointState, 'arm/encoder_counts', 10)
        self._pub_pots = self.create_publisher(Int32MultiArray, 'arm/pot_raw', 10)
        self._sub_commands = self.create_subscription(
            JointState,
            'arm_joint_commands',
            self._command_callback,
            10,
        )

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._read_loop)
        self.get_logger().info('STM32SerialBridge node started.')

    # ------------------------------------------------------------------
    # Serial read loop
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Read one telemetry line from the STM32 and publish it."""
        if self._serial is None or not self._serial.is_open:
            return
        try:
            raw = self._serial.readline()
            line = raw.decode('ascii', errors='ignore').strip()
            if line:
                self._parse_and_publish(line)
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read error: {exc}')

    def _parse_and_publish(self, line: str) -> None:
        """Parse STM32 telemetry line and publish to ROS topics.

        Expected format::

            "ENC:<enc_base>,<enc_wrist1>,<enc_wrist2>;POT:<pot_hum>,<pot_fore>"
        """
        try:
            enc_part, pot_part = line.split(';')
            enc_vals = enc_part.lstrip('ENC:').split(',')
            pot_vals = pot_part.lstrip('POT:').split(',')

            enc_base = int(enc_vals[0])
            enc_wrist1 = int(enc_vals[1])
            enc_wrist2 = int(enc_vals[2])
            pot_hum = int(pot_vals[0])
            pot_fore = int(pot_vals[1])

        except (ValueError, IndexError):
            self.get_logger().debug(f'Could not parse STM32 line: {line!r}')
            return

        # Publish encoder counts as a JointState (positions in counts)
        enc_msg = JointState()
        enc_msg.header.stamp = self.get_clock().now().to_msg()
        enc_msg.name = [JOINT_BASE, JOINT_WRIST_PITCH, JOINT_WRIST_ROLL]
        enc_msg.position = [float(enc_base), float(enc_wrist1), float(enc_wrist2)]
        self._pub_encoders.publish(enc_msg)

        # Publish raw ADC values
        pot_msg = Int32MultiArray()
        pot_msg.data = [pot_hum, pot_fore]
        self._pub_pots.publish(pot_msg)

    # ------------------------------------------------------------------
    # Command forwarding
    # ------------------------------------------------------------------

    def _command_callback(self, msg: JointState) -> None:
        """Forward desired joint positions as encoder counts to the STM32."""
        if self._serial is None or not self._serial.is_open:
            return

        positions = {name: pos for name, pos in zip(msg.name, msg.position)}

        base_rad = positions.get(JOINT_BASE, 0.0)
        pitch_rad = positions.get(JOINT_WRIST_PITCH, 0.0)
        roll_rad = positions.get(JOINT_WRIST_ROLL, 0.0)

        if self._signal is not None:
            base_enc = self._signal.radians_to_encoder(base_rad)
            enc_m1, enc_m2 = SignalUtils.pitch_roll_to_differential(
                pitch_rad, roll_rad,
                cpr=self._signal.encoder_cpr,
                gear_ratio=self._signal.gear_ratio,
            )
        else:
            base_enc = int(base_rad)
            enc_m1 = int(pitch_rad)
            enc_m2 = int(roll_rad)

        cmd = f'CMD:{base_enc},{enc_m1},{enc_m2}\n'
        try:
            self._serial.write(cmd.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
