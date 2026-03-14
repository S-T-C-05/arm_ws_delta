"""Hardware controller node for the robotic arm.

This node is the central bridge between ROS 2 and the physical hardware:

Subscriptions:
  /arm_joint_commands  (sensor_msgs/JointState) - desired joint positions from
                        MoveIt / teleop / autonomous planners.

Publications:
  /joint_states        (sensor_msgs/JointState) - actual joint positions from
                        sensor feedback (encoders + potentiometers).

The node:
1. Reads sensor feedback via SensorReader at ~50 Hz.
2. Converts raw encoder / ADC values to radians using SignalUtils.
3. Publishes /joint_states so RViz and MoveIt reflect real hardware state.
4. Receives desired positions on /arm_joint_commands.
5. Converts desired radians to motor commands and sends them to RoboClaw /
   SparkFun via their respective drivers.

Parameters (YAML or command line):
  roboclaw_port   (str)  '/dev/ttyACM0'
  roboclaw_baud   (int)  115200
  sparkfun_port   (str)  '/dev/ttyUSB0'
  sparkfun_baud   (int)  115200
  sensor_port     (str)  '/dev/ttyACM1'
  sensor_baud     (int)  115200
  publish_rate_hz (float) 50.0
  encoder_cpr     (int)  4096
  gear_ratio      (float) 1.0
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_hardware_interface.roboclaw_driver import RoboClawDriver, CHANNEL_M1, CHANNEL_M2
from arm_hardware_interface.sparkfun_driver import SparkFunDriver, CHANNEL_BASE
from arm_hardware_interface.sensor_reader import SensorReader
from arm_hardware_interface.signal_utils import SignalUtils


# Joint name constants (must match the URDF)
JOINT_BASE = 'bracket_joint'
JOINT_HUMERUS = 'humerus_low_joint'
JOINT_FOREARM = 'forearm_low_joint'
JOINT_WRIST_PITCH = 'ubracket_joint'
JOINT_WRIST_ROLL = 'endeffector_joint'

ALL_JOINTS = [JOINT_BASE, JOINT_HUMERUS, JOINT_FOREARM, JOINT_WRIST_PITCH, JOINT_WRIST_ROLL]


class HardwareControllerNode(Node):
    """ROS 2 node that bridges joint commands to hardware drivers."""

    def __init__(self) -> None:
        super().__init__('hardware_controller')

        # ---- Declare parameters ----
        self.declare_parameter('roboclaw_port', '/dev/ttyACM0')
        self.declare_parameter('roboclaw_baud', 115200)
        self.declare_parameter('sparkfun_port', '/dev/ttyUSB0')
        self.declare_parameter('sparkfun_baud', 115200)
        self.declare_parameter('sensor_port', '/dev/ttyACM1')
        self.declare_parameter('sensor_baud', 115200)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('encoder_cpr', 4096)
        self.declare_parameter('gear_ratio', 1.0)

        # ---- Instantiate drivers ----
        rc_port = self.get_parameter('roboclaw_port').get_parameter_value().string_value
        rc_baud = self.get_parameter('roboclaw_baud').get_parameter_value().integer_value
        sf_port = self.get_parameter('sparkfun_port').get_parameter_value().string_value
        sf_baud = self.get_parameter('sparkfun_baud').get_parameter_value().integer_value
        sr_port = self.get_parameter('sensor_port').get_parameter_value().string_value
        sr_baud = self.get_parameter('sensor_baud').get_parameter_value().integer_value
        cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        gr = self.get_parameter('gear_ratio').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self._signal = SignalUtils(encoder_cpr=cpr, gear_ratio=gr)

        self._roboclaw = RoboClawDriver(port=rc_port, baudrate=rc_baud)
        self._sparkfun = SparkFunDriver(port=sf_port, baudrate=sf_baud)
        self._sensor_reader = SensorReader(port=sr_port, baudrate=sr_baud)

        # ---- Connect to hardware ----
        self._hw_active = False
        if self._roboclaw.connect():
            self.get_logger().info(f'RoboClaw connected on {rc_port}')
            self._hw_active = True
        else:
            self.get_logger().warn(f'RoboClaw not available on {rc_port} - running in degraded mode')

        if self._sparkfun.connect():
            self.get_logger().info(f'SparkFun connected on {sf_port}')
        else:
            self.get_logger().warn(f'SparkFun not available on {sf_port} - running in degraded mode')

        if self._sensor_reader.connect():
            self.get_logger().info(f'Sensor reader connected on {sr_port}')
        else:
            self.get_logger().warn(f'Sensor reader not available on {sr_port} - using zero feedback')

        # ---- State ----
        self._cmd_positions = {j: 0.0 for j in ALL_JOINTS}
        self._actual_positions = {j: 0.0 for j in ALL_JOINTS}

        # ---- ROS interfaces ----
        self._pub_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        self._sub_commands = self.create_subscription(
            JointState,
            'arm_joint_commands',
            self._command_callback,
            10,
        )

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._control_loop)
        self.get_logger().info('HardwareControllerNode started.')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _command_callback(self, msg: JointState) -> None:
        """Store the latest commanded joint positions."""
        for idx, name in enumerate(msg.name):
            if name in self._cmd_positions and idx < len(msg.position):
                self._cmd_positions[name] = msg.position[idx]
        self._send_commands_to_hardware()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Read sensors, update state, publish joint_states."""
        self._read_sensor_feedback()
        self._publish_joint_states()

    def _read_sensor_feedback(self) -> None:
        """Read encoder and potentiometer data and update actual positions."""
        if self._sensor_reader.is_connected():
            self._sensor_reader.read_once()

            # Base motor (encoder via STM32)
            self._actual_positions[JOINT_BASE] = self._signal.encoder_to_radians(
                self._sensor_reader.enc_base
            )

            # Wrist differential
            pitch, roll = SignalUtils.differential_to_pitch_roll(
                self._sensor_reader.enc_wrist1,
                self._sensor_reader.enc_wrist2,
                cpr=self._signal.encoder_cpr,
                gear_ratio=self._signal.gear_ratio,
            )
            self._actual_positions[JOINT_WRIST_PITCH] = pitch
            self._actual_positions[JOINT_WRIST_ROLL] = roll

            # Linear actuators (potentiometers)
            self._actual_positions[JOINT_HUMERUS] = self._signal.adc_to_radians(
                self._sensor_reader.pot_humerus_raw
            )
            self._actual_positions[JOINT_FOREARM] = self._signal.adc_to_radians(
                self._sensor_reader.pot_forearm_raw
            )

    def _send_commands_to_hardware(self) -> None:
        """Convert desired joint positions to motor commands."""
        # Base motor (SparkFun, velocity-mode via proportional control)
        if self._sparkfun.is_connected():
            base_error = self._cmd_positions[JOINT_BASE] - self._actual_positions[JOINT_BASE]
            # Simple P controller: gain 200 → velocity in [-255, 255]
            base_vel = int(max(-255, min(255, base_error * 200.0)))
            self._sparkfun.set_velocity(CHANNEL_BASE, base_vel)

        # Wrist differential (RoboClaw, position-mode with onboard PID)
        if self._roboclaw.is_connected():
            enc_m1, enc_m2 = SignalUtils.pitch_roll_to_differential(
                self._cmd_positions[JOINT_WRIST_PITCH],
                self._cmd_positions[JOINT_WRIST_ROLL],
                cpr=self._signal.encoder_cpr,
                gear_ratio=self._signal.gear_ratio,
            )
            self._roboclaw.set_position(CHANNEL_M1, enc_m1)
            self._roboclaw.set_position(CHANNEL_M2, enc_m2)

    def _publish_joint_states(self) -> None:
        """Publish the current actual joint positions to /joint_states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self._actual_positions[j] for j in ALL_JOINTS]
        self._pub_joint_states.publish(msg)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        """Graceful shutdown: stop all motors and close connections."""
        self.get_logger().info('Shutting down hardware controller - stopping motors.')
        try:
            self._roboclaw.stop()
            self._sparkfun.stop()
        except Exception:
            pass
        self._roboclaw.disconnect()
        self._sparkfun.disconnect()
        self._sensor_reader.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
