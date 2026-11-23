"""ROS 2 side of the GUI: publishers, service clients and data buffers.

The node is spun in a background thread. Callbacks only store data; the Qt
widgets read it periodically (thread safe via a lock).
"""

import threading
import time
from collections import deque
from typing import Dict, List, Optional, Sequence

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from robothand_msgs.msg import HandState, StrainGauges
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']


class History:
    """Ring buffer of (time, values) samples."""

    def __init__(self, seconds: float = 20.0, max_samples: int = 4000):
        self.seconds = seconds
        self.t = deque(maxlen=max_samples)
        self.v = deque(maxlen=max_samples)

    def append(self, t: float, values: Sequence[float]) -> None:
        self.t.append(t)
        self.v.append(list(values))
        while self.t and t - self.t[0] > self.seconds:
            self.t.popleft()
            self.v.popleft()

    def arrays(self):
        if not self.t:
            return np.zeros(0), np.zeros((0, 0))
        n = min(len(v) for v in self.v)
        return np.array(self.t), np.array([v[:n] for v in self.v])


class GuiNode(Node):

    def __init__(self):
        super().__init__('robothand_gui')
        self.declare_parameter('history_s', 20.0)
        history_s = float(self.get_parameter('history_s').value)

        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.joint_positions: Dict[str, float] = {}
        self.joint_stamp = 0.0
        self.joint_history = History(history_s)
        self.state: Optional[HandState] = None
        self.state_stamp = 0.0
        self.gauges: Optional[StrainGauges] = None
        self.gauge_history = History(history_s)

        self.closure_pub = self.create_publisher(Float64MultiArray, 'robothand/closure_command', 10)
        self.joint_pub = self.create_publisher(JointState, 'robothand/joint_command', 10)

        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)
        self.create_subscription(HandState, 'robothand/state', self._on_state, 10)
        self.create_subscription(StrainGauges, 'robothand/strain_gauges', self._on_gauges, 10)

        self.enable_cli = self.create_client(SetBool, 'robothand/enable')
        self.home_cli = self.create_client(Trigger, 'robothand/home')
        self.tare_cli = self.create_client(Trigger, 'robothand/tare')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def now(self) -> float:
        return time.monotonic() - self.t0

    # ---------------------------------------------------------- callbacks
    def _on_joint_states(self, msg: JointState) -> None:
        t = self.now()
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.joint_positions[name] = pos
            self.joint_stamp = t
            self.joint_history.append(
                t, [self.joint_positions.get(f + '_mcp_joint', 0.0) for f in FINGERS])

    def _on_state(self, msg: HandState) -> None:
        with self.lock:
            self.state = msg
            self.state_stamp = self.now()

    def _on_gauges(self, msg: StrainGauges) -> None:
        t = self.now()
        with self.lock:
            self.gauges = msg
            self.gauge_history.append(t, msg.values)

    # ---------------------------------------------------------- data access
    def snapshot_joints(self) -> Dict[str, float]:
        with self.lock:
            return dict(self.joint_positions)

    def joint_series(self):
        with self.lock:
            return self.joint_history.arrays()

    def gauge_series(self):
        with self.lock:
            names = list(self.gauges.names) if self.gauges else []
            unit = self.gauges.unit if self.gauges else ''
            t, v = self.gauge_history.arrays()
        return names, unit, t, v

    def latest_state(self):
        with self.lock:
            return self.state, self.now() - self.state_stamp if self.state else None

    def lookup(self, target: str, source: str) -> Optional[np.ndarray]:
        """Position of frame `source` in frame `target` from TF (or None)."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        tr = tf.transform.translation
        return np.array([tr.x, tr.y, tr.z])

    # ---------------------------------------------------------- commands
    def send_closures(self, closures: Sequence[float]) -> None:
        self.closure_pub.publish(Float64MultiArray(data=[float(c) for c in closures]))

    def send_joints(self, names: List[str], positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        self.joint_pub.publish(msg)

    def call_enable(self, enable: bool) -> bool:
        if not self.enable_cli.service_is_ready():
            return False
        self.enable_cli.call_async(SetBool.Request(data=enable))
        return True

    def call_trigger(self, client) -> bool:
        if not client.service_is_ready():
            return False
        client.call_async(Trigger.Request())
        return True
