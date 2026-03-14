"""Mock hardware controller node (simulation / testing without physical hardware).

Behaves identically to HardwareControllerNode but uses simulated sensor
feedback instead of real drivers.  The mock controller:

- Accepts joint commands on /arm_joint_commands.
- Simulates first-order lag dynamics to produce realistic joint_states.
- Publishes /joint_states at the configured rate.
- Requires **no** serial ports or physical hardware.

Use this node to:
  - Test MoveIt / teleop integration on a development machine.
  - Validate the full ROS 2 pipeline before connecting hardware.
  - Run CI/CD automated tests.

Parameters (YAML or command line):
  publish_rate_hz (float) 50.0
  time_constant   (float) 0.1   - First-order lag time constant (seconds).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Joint name constants (must match the URDF)
JOINT_BASE = 'bracket_joint'
JOINT_HUMERUS = 'humerus_low_joint'
JOINT_FOREARM = 'forearm_low_joint'
JOINT_WRIST_PITCH = 'ubracket_joint'
JOINT_WRIST_ROLL = 'endeffector_joint'

ALL_JOINTS = [JOINT_BASE, JOINT_HUMERUS, JOINT_FOREARM, JOINT_WRIST_PITCH, JOINT_WRIST_ROLL]


class MockHardwareController(Node):
    """Simulated hardware controller for development and testing.

    Implements first-order lag dynamics so that joint_states smoothly
    converge toward the commanded positions, mimicking real motor behavior.
    """

    def __init__(self) -> None:
        super().__init__('mock_hardware_controller')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('time_constant', 0.1)

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._tau = self.get_parameter('time_constant').get_parameter_value().double_value
        self._dt = 1.0 / rate

        self._cmd_positions = {j: 0.0 for j in ALL_JOINTS}
        self._actual_positions = {j: 0.0 for j in ALL_JOINTS}

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._sub = self.create_subscription(
            JointState,
            'arm_joint_commands',
            self._command_callback,
            10,
        )

        self.create_timer(self._dt, self._control_loop)
        self.get_logger().info('MockHardwareController started (no hardware required).')

    def _command_callback(self, msg: JointState) -> None:
        """Store the latest commanded joint positions."""
        for idx, name in enumerate(msg.name):
            if name in self._cmd_positions and idx < len(msg.position):
                self._cmd_positions[name] = msg.position[idx]

    def _control_loop(self) -> None:
        """Advance simulation one time step and publish joint_states."""
        alpha = self._dt / (self._tau + self._dt)
        for joint in ALL_JOINTS:
            cmd = self._cmd_positions[joint]
            actual = self._actual_positions[joint]
            self._actual_positions[joint] = actual + alpha * (cmd - actual)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self._actual_positions[j] for j in ALL_JOINTS]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockHardwareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
