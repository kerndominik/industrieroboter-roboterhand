"""Mapping between servo angles, finger closure and URDF joint angles.

Each finger is driven by one servo (tendon). The firmware works with servo
angles in degrees, ROS with joint angles in radians:

    closure c in [0, 1]          0 = finger open, 1 = finger closed
    servo   = servo_open + c * (servo_closed - servo_open)            [deg]
    q_mcp   = c * joint_closed                                         [rad]

The remaining (coupled) finger joints follow from the <mimic> tags of the URDF:
    q = multiplier * q_mimicked + offset
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


class ActuatorMap:
    """Linear servo <-> closure <-> joint angle mapping of the five fingers."""

    def __init__(self, joint_names: Sequence[str], servo_open_deg: Sequence[float],
                 servo_closed_deg: Sequence[float], joint_closed_rad: Sequence[float]):
        n = len(joint_names)
        if not (len(servo_open_deg) == len(servo_closed_deg) == len(joint_closed_rad) == n):
            raise ValueError('all actuator parameter lists must have the same length')
        for i in range(n):
            if servo_open_deg[i] == servo_closed_deg[i]:
                raise ValueError(f'servo_open_deg == servo_closed_deg for {joint_names[i]}')
            if joint_closed_rad[i] == 0.0:
                raise ValueError(f'joint_closed_rad must not be 0 for {joint_names[i]}')
        self.joint_names = list(joint_names)
        self.servo_open = [float(v) for v in servo_open_deg]
        self.servo_closed = [float(v) for v in servo_closed_deg]
        self.joint_closed = [float(v) for v in joint_closed_rad]

    def __len__(self) -> int:
        return len(self.joint_names)

    def index(self, joint_name: str) -> Optional[int]:
        try:
            return self.joint_names.index(joint_name)
        except ValueError:
            return None

    def closure_to_servo(self, i: int, closure: float) -> float:
        c = _clamp(closure, 0.0, 1.0)
        return self.servo_open[i] + c * (self.servo_closed[i] - self.servo_open[i])

    def servo_to_closure(self, i: int, servo_deg: float) -> float:
        c = (servo_deg - self.servo_open[i]) / (self.servo_closed[i] - self.servo_open[i])
        return _clamp(c, 0.0, 1.0)

    def joint_to_closure(self, i: int, q: float) -> float:
        return _clamp(q / self.joint_closed[i], 0.0, 1.0)

    def closure_to_joint(self, i: int, closure: float) -> float:
        return _clamp(closure, 0.0, 1.0) * self.joint_closed[i]


@dataclass
class JointInfo:
    name: str
    lower: float = -math.inf
    upper: float = math.inf
    mimic_of: Optional[str] = None
    multiplier: float = 1.0
    offset: float = 0.0


class JointModel:
    """Movable joints of the URDF and their mimic relations."""

    def __init__(self, joints: Optional[List[JointInfo]] = None):
        self.joints: Dict[str, JointInfo] = {j.name: j for j in (joints or [])}
        self.order: List[str] = [j.name for j in (joints or [])]

    @classmethod
    def from_urdf(cls, urdf_xml: str) -> 'JointModel':
        root = ET.fromstring(urdf_xml)
        joints = []
        for el in root.findall('joint'):
            if el.get('type') in ('fixed', 'floating', 'planar', None):
                continue
            info = JointInfo(el.get('name'))
            limit = el.find('limit')
            if limit is not None and el.get('type') != 'continuous':
                info.lower = float(limit.get('lower', '-inf'))
                info.upper = float(limit.get('upper', 'inf'))
            mimic = el.find('mimic')
            if mimic is not None:
                info.mimic_of = mimic.get('joint')
                info.multiplier = float(mimic.get('multiplier', '1.0'))
                info.offset = float(mimic.get('offset', '0.0'))
            joints.append(info)
        return cls(joints)

    def __contains__(self, name: str) -> bool:
        return name in self.joints

    def is_mimic(self, name: str) -> bool:
        return name in self.joints and self.joints[name].mimic_of is not None

    def clamp(self, name: str, value: float) -> float:
        info = self.joints.get(name)
        if info is None:
            return value
        return _clamp(value, info.lower, info.upper)

    def expand(self, values: Dict[str, float]) -> Tuple[List[str], List[float]]:
        """Return all joint names/positions; mimic joints are computed.

        Independent joints missing in ``values`` are reported as 0.0. If the
        model is empty (no URDF yet) only the given values are returned.
        """
        if not self.joints:
            return list(values.keys()), list(values.values())
        resolved: Dict[str, float] = {}

        def resolve(name: str, depth: int = 0) -> float:
            if name in resolved:
                return resolved[name]
            info = self.joints.get(name)
            if info is None or info.mimic_of is None or depth > 10:
                value = values.get(name, 0.0)
            else:
                value = info.multiplier * resolve(info.mimic_of, depth + 1) + info.offset
            resolved[name] = value
            return value

        names = list(self.order)
        return names, [resolve(n) for n in names]
