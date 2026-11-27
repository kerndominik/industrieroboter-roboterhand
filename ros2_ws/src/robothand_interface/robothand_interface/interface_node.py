"""ROS 2 hardware interface of the robot hand.

Bridges ROS 2 topics/services and the microcontroller (Arduino Mega 2560 or
ESP32) that drives the five finger servos and reads the strain gauges.

Subscribed topics
  robothand/joint_command    sensor_msgs/JointState      target joint angles [rad]
                             (actuated *_mcp_joint and passive joints, partial
                             messages allowed)
  robothand/closure_command  std_msgs/Float64MultiArray  5 x closure 0..1
                             (thumb, index, middle, ring, little; NaN = keep)
  robothand/servo_command    std_msgs/Float64MultiArray  5 x raw servo angle [deg]
                             (for calibration; NaN = keep)
  robot_description          std_msgs/String             URDF (for mimic joints)

Published topics
  joint_states               sensor_msgs/JointState      all URDF joints -> robot_state_publisher
  robothand/state            robothand_msgs/HandState
  robothand/strain_gauges    robothand_msgs/StrainGauges

Services
  robothand/enable           std_srvs/SetBool   attach (true) / detach (false) servos
  robothand/home             std_srvs/Trigger   open all fingers (closure 0)
  robothand/tare             std_srvs/Trigger   zero the strain gauges
"""

import math
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from robothand_msgs.msg import HandState, StrainGauges
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from . import protocol
from .hand_model import ActuatorMap, JointModel
from .links import EmulatedMcu, LinkError, SerialLink

FINGERS = list(protocol.FINGERS)


class RobotHandInterface(Node):

    def __init__(self):
        super().__init__('robothand_interface')

        # ---------------- parameters ----------------
        p = self._declare
        self.port = p('port', '/dev/ttyUSB0',
                      'Serial port or pyserial URL (e.g. socket://localhost:5555)')
        self.baudrate = p('baudrate', 115200, 'Serial baud rate')
        self.simulate = p('simulate', False, 'Use the emulated microcontroller')
        self.actuated_joints = p('actuated_joints', [f + '_mcp_joint' for f in FINGERS],
                                 'URDF joints driven by servo 0..4 (thumb..little)')
        servo_open = p('servo_open_deg', [10.0] * 5, 'Servo angle of the open finger [deg]')
        servo_closed = p('servo_closed_deg', [170.0] * 5, 'Servo angle of the closed finger [deg]')
        joint_closed = p('joint_closed_rad', [1.5708, 1.4835, 1.4835, 1.4835, 1.4835],
                         'Angle of the actuated joint when the finger is closed [rad]')
        passive_names = p('passive_joints', ['thumb_rotation_joint'],
                          'Joints without servo (set manually / via joint_command)')
        passive_pos = p('passive_joint_positions', [0.2863], 'Initial passive joint angles [rad]')
        self.max_speed = p('max_speed_deg_s', 180.0, 'Max. servo speed [deg/s], 0 = no ramp')
        self.telemetry_hz = p('telemetry_rate_hz', 50.0, 'Firmware telemetry rate [Hz]')
        self.publish_hz = p('publish_rate_hz', 50.0, 'joint_states publish rate [Hz]')
        self.command_hz = p('command_rate_hz', 50.0, 'Max. rate of servo commands [Hz]')
        self.gauge_names = p('gauge_names', list(FINGERS), 'Names of the strain gauges')
        self.gauge_scale = p('gauge_scale', [1.0] * 5, 'value = raw * scale + offset')
        self.gauge_offset = p('gauge_offset', [0.0] * 5, 'value = raw * scale + offset')
        self.gauge_unit = p('gauge_unit', 'counts', 'Unit of the calibrated gauge values')
        self.disable_on_shutdown = p('disable_on_shutdown', True,
                                     'Detach the servos (limp) when the node stops')
        self.reconnect_s = p('reconnect_period_s', 2.0, 'Retry period if the port is not available')

        self.actuators = ActuatorMap(self.actuated_joints, servo_open, servo_closed, joint_closed)
        if len(passive_names) != len(passive_pos):
            raise ValueError('passive_joints and passive_joint_positions differ in length')
        self.passive = dict(zip(passive_names, passive_pos))
        self.joint_model = JointModel()

        # ---------------- state ----------------
        n = len(self.actuators)
        self.servo_target: List[float] = [self.actuators.closure_to_servo(i, 0.0) for i in range(n)]
        self.servo_current: Optional[List[float]] = None
        self.servos_enabled = False
        self.commanded = False          # a target was received at least once
        self.command_dirty = False
        self.output_enabled = True      # False after robothand/enable(false)
        self.last_command_sent = 0.0
        self.connected = False
        self.firmware = ''
        self.opened_at = 0.0
        self.last_rx = 0.0
        self.last_info_request = 0.0
        self.last_open_attempt = -1e9
        self._warned = set()

        if self.simulate:
            self.link = EmulatedMcu(telemetry_hz=self.telemetry_hz,
                                    max_speed_deg_s=self.max_speed,
                                    servo_open_deg=servo_open, servo_closed_deg=servo_closed)
        else:
            self.link = SerialLink(self.port, self.baudrate)

        # ---------------- ROS interfaces ----------------
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.state_pub = self.create_publisher(HandState, 'robothand/state', 10)
        self.gauge_pub = self.create_publisher(StrainGauges, 'robothand/strain_gauges', 10)

        self.create_subscription(JointState, 'robothand/joint_command', self._on_joint_command, 10)
        self.create_subscription(Float64MultiArray, 'robothand/closure_command',
                                 self._on_closure_command, 10)
        self.create_subscription(Float64MultiArray, 'robothand/servo_command',
                                 self._on_servo_command, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'robot_description', self._on_robot_description, latched)

        self.create_service(SetBool, 'robothand/enable', self._srv_enable)
        self.create_service(Trigger, 'robothand/home', self._srv_home)
        self.create_service(Trigger, 'robothand/tare', self._srv_tare)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(0.01, self._poll)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._publish_state)

        mode = 'SIMULATION (emulated microcontroller)' if self.simulate else f'port {self.port}'
        self.get_logger().info(f'robothand_interface started - {mode}')

    # ------------------------------------------------------------------ helpers
    def _declare(self, name, default, description=''):
        self.declare_parameter(name, default, ParameterDescriptor(description=description))
        return self.get_parameter(name).value

    def _warn_once(self, key: str, msg: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(msg)

    def _send(self, line: str) -> bool:
        if not self.link.is_open:
            return False
        try:
            self.link.write_line(line)
            return True
        except LinkError as exc:
            self._on_link_lost(str(exc))
            return False

    def _on_link_lost(self, reason: str) -> None:
        if self.connected or self.opened_at:
            self.get_logger().error(f'connection lost: {reason}')
        self.link.close()
        self.connected = False
        self.opened_at = 0.0
        self.servo_current = None

    def _set_target(self, i: int, servo_deg: float) -> None:
        self.servo_target[i] = servo_deg
        self.commanded = True
        self.command_dirty = True

    # ----------------------------------------------------------- main loop
    def _poll(self) -> None:
        now = time.monotonic()
        if not self.link.is_open:
            if now - self.last_open_attempt >= self.reconnect_s:
                self.last_open_attempt = now
                self._try_open(now)
            return

        try:
            lines = self.link.read_lines()
        except LinkError as exc:
            self._on_link_lost(str(exc))
            return
        for line in lines:
            self._handle_line(line, now)

        # handshake: ask for the board info until it answers
        if not self.connected and now - self.opened_at > 2.5 and now - self.last_info_request > 1.0:
            self.last_info_request = now
            self._send(protocol.cmd_info())
        # connection watchdog (only if telemetry is expected)
        if self.connected and self.telemetry_hz > 0 and now - self.last_rx > 2.0:
            self.get_logger().warn('no data from the microcontroller for 2 s')
            self.connected = False

        # targets are only sent while enabled (the firmware attaches the servos on S)
        if (self.command_dirty and self.connected and self.output_enabled
                and now - self.last_command_sent >= 1.0 / max(self.command_hz, 1.0)):
            if self._send(protocol.cmd_set_all(self.servo_target)):
                self.command_dirty = False
                self.last_command_sent = now

    def _try_open(self, now: float) -> None:
        try:
            self.link.open()
        except LinkError as exc:
            self._warn_once('open', f'{exc} - retrying every {self.reconnect_s:.0f} s')
            return
        self._warned.discard('open')
        self.opened_at = now
        self.last_rx = now
        self.last_info_request = now
        self.get_logger().info(f'port opened ({"emulated" if self.simulate else self.port}),'
                               ' waiting for the board ...')

    def _handle_line(self, line: str, now: float) -> None:
        msg = protocol.parse_line(line)
        if msg is None:
            if line.strip():
                self.get_logger().debug(f'ignored line: {line!r}')
            return
        self.last_rx = now
        if isinstance(msg, protocol.InfoMessage):
            self._on_info(msg)
        elif isinstance(msg, protocol.ServoStatus):
            if not self.connected and now - self.last_info_request > 1.0:
                # data without handshake (no boot message / after a dropout):
                # ask for the info, the answer (re-)applies the settings
                self.last_info_request = now
                self._send(protocol.cmd_info())
            if len(msg.angles_deg) == len(self.actuators):
                self.servo_current = msg.angles_deg
                if not self.commanded:
                    self.servo_target = list(msg.angles_deg)
            self.servos_enabled = msg.enabled
        elif isinstance(msg, protocol.GaugeStatus):
            self._publish_gauges(msg)
        elif isinstance(msg, protocol.ErrorMessage):
            self.get_logger().warn(f'firmware error: {msg.message}')
        elif isinstance(msg, protocol.LogMessage):
            self.get_logger().info(f'firmware: {msg.text}')

    def _on_info(self, info: protocol.InfoMessage) -> None:
        self.firmware = info.description
        self.connected = True
        self.get_logger().info(f'connected to {self.firmware} ({info.n_servos} servos, '
                               f'{info.n_gauges} gauges, source {info.gauge_source})')
        if info.n_servos != len(self.actuators):
            self.get_logger().error(f'firmware has {info.n_servos} servos, '
                                    f'expected {len(self.actuators)}')
        # (re-)apply settings - the board may just have been reset
        self._send(protocol.cmd_speed(self.max_speed))
        self._send(protocol.cmd_rate(self.telemetry_hz))
        if self.commanded:
            self.command_dirty = True

    # ----------------------------------------------------------- callbacks
    def _on_robot_description(self, msg: String) -> None:
        try:
            self.joint_model = JointModel.from_urdf(msg.data)
        except Exception as exc:  # noqa: BLE001 - malformed URDF must not kill the node
            self.get_logger().error(f'cannot parse robot_description: {exc}')
            return
        missing = [j for j in list(self.actuated_joints) + list(self.passive)
                   if j not in self.joint_model]
        if missing:
            self.get_logger().error(f'joints not found in the URDF: {missing}')
        self.get_logger().info(f'robot_description received ({len(self.joint_model.order)} joints)')

    def _on_joint_command(self, msg: JointState) -> None:
        if len(msg.position) < len(msg.name):
            self.get_logger().warn('joint_command: fewer positions than names - ignored')
            return
        for name, q in zip(msg.name, msg.position):
            if math.isnan(q):
                continue
            i = self.actuators.index(name)
            if i is not None:
                closure = self.actuators.joint_to_closure(i, q)
                self._set_target(i, self.actuators.closure_to_servo(i, closure))
            elif name in self.passive:
                self.passive[name] = self.joint_model.clamp(name, q)
            elif self.joint_model.is_mimic(name):
                self._warn_once('mimic:' + name,
                                f'joint_command: {name} is coupled (mimic) and cannot be set')
            else:
                self._warn_once('unknown:' + name, f'joint_command: unknown joint {name}')

    def _on_closure_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.actuators):
            self.get_logger().warn(f'closure_command needs {len(self.actuators)} values')
            return
        for i, c in enumerate(msg.data):
            if not math.isnan(c):
                self._set_target(i, self.actuators.closure_to_servo(i, c))

    def _on_servo_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.actuators):
            self.get_logger().warn(f'servo_command needs {len(self.actuators)} values')
            return
        for i, deg in enumerate(msg.data):
            if not math.isnan(deg):
                self._set_target(i, float(deg))

    def _srv_enable(self, request, response):
        # Disabling is remembered even without connection: no targets are sent
        # until the servos are enabled again (then the latest target is sent).
        self.output_enabled = bool(request.data)
        ok = self._send(protocol.cmd_enable(request.data))
        if request.data:
            self.commanded = True  # enabling without a target: hold the current position
            self.command_dirty = True
        response.success = ok
        state = 'enabled' if request.data else 'disabled'
        response.message = f'servos {state}' if ok else f'not connected (servos {state} on connect)'
        return response

    def _srv_home(self, request, response):
        for i in range(len(self.actuators)):
            self._set_target(i, self.actuators.closure_to_servo(i, 0.0))
        response.success = True
        response.message = 'opening all fingers'
        return response

    def _srv_tare(self, request, response):
        ok = self._send(protocol.cmd_tare())
        response.success = ok
        response.message = 'strain gauges tared' if ok else 'not connected'
        return response

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == 'max_speed_deg_s':
                self.max_speed = float(param.value)
                self._send(protocol.cmd_speed(self.max_speed))
            elif param.name == 'gauge_scale':
                self.gauge_scale = list(param.value)
            elif param.name == 'gauge_offset':
                self.gauge_offset = list(param.value)
        return SetParametersResult(successful=True)

    # ----------------------------------------------------------- publishing
    def _closures(self) -> List[float]:
        servo = self.servo_current if self.servo_current is not None else self.servo_target
        return [self.actuators.servo_to_closure(i, s) for i, s in enumerate(servo)]

    def _publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        closures = self._closures()
        values = {name: self.actuators.closure_to_joint(i, c)
                  for i, (name, c) in enumerate(zip(self.actuated_joints, closures))}
        values.update(self.passive)
        names, positions = self.joint_model.expand(values)

        js = JointState()
        js.header.stamp = stamp
        js.name = names
        js.position = [float(v) for v in positions]
        self.joint_state_pub.publish(js)

        st = HandState()
        st.header.stamp = stamp
        st.connected = self.connected
        st.servos_enabled = self.servos_enabled
        st.firmware = self.firmware
        st.fingers = FINGERS[:len(self.actuators)]
        st.servo_angles_deg = [float(v) for v in (self.servo_current or [])]
        st.servo_targets_deg = [float(v) for v in self.servo_target]
        st.closure = [float(c) for c in closures]
        self.state_pub.publish(st)

    def _publish_gauges(self, msg: protocol.GaugeStatus) -> None:
        n = len(msg.values)
        names = list(self.gauge_names[:n]) + [f'gauge{i}' for i in range(len(self.gauge_names), n)]
        scale = list(self.gauge_scale[:n]) + [1.0] * max(0, n - len(self.gauge_scale))
        offset = list(self.gauge_offset[:n]) + [0.0] * max(0, n - len(self.gauge_offset))
        out = StrainGauges()
        out.header.stamp = self.get_clock().now().to_msg()
        out.names = names
        out.raw = [float(v) for v in msg.values]
        out.values = [float(v) * s + o for v, s, o in zip(msg.values, scale, offset)]
        out.unit = self.gauge_unit
        self.gauge_pub.publish(out)

    def shutdown(self) -> None:
        if self.disable_on_shutdown and self.link.is_open:
            self._send(protocol.cmd_enable(False))
            if rclpy.ok():  # the ROS context may already be shut down (Ctrl+C)
                self.get_logger().info('servos disabled')
        self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = RobotHandInterface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
