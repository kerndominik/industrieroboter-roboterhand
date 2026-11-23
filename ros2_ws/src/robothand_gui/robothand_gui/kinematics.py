"""Forward kinematics of the robot hand with Denavit-Hartenberg parameters.

Standard (distal) DH convention as in the lecture:

    A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)

        | cos(th)  -sin(th)cos(al)   sin(th)sin(al)   a cos(th) |
        | sin(th)   cos(th)cos(al)  -cos(th)sin(al)   a sin(th) |
        |   0         sin(al)          cos(al)            d     |
        |   0           0                0                1     |

    theta_i = q_i + theta_offset_i

    T_palm_tip = T_palm_0 * A_1 * ... * A_n

The parameter file is robothand_description/config/dh_parameters.yaml.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np
import yaml


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Homogeneous transform A_i of one DH row."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


@dataclass
class DHRow:
    theta_offset: float
    d: float
    a: float
    alpha: float


@dataclass
class FingerChain:
    name: str
    joints: List[str]
    rows: List[DHRow]
    base_transform: np.ndarray
    tip_frame: str

    def thetas(self, q: Sequence[float]) -> List[float]:
        return [qi + row.theta_offset for qi, row in zip(q, self.rows)]

    def link_transforms(self, q: Sequence[float]) -> List[np.ndarray]:
        """A_1 .. A_n for the joint angles q."""
        return [dh_matrix(th, r.d, r.a, r.alpha) for th, r in zip(self.thetas(q), self.rows)]

    def frames(self, q: Sequence[float]) -> List[np.ndarray]:
        """Poses of the DH frames 0..n in the base frame (palm_link)."""
        out = [self.base_transform.copy()]
        for a in self.link_transforms(q):
            out.append(out[-1] @ a)
        return out

    def forward(self, q: Sequence[float]) -> np.ndarray:
        """T_palm_tip."""
        return self.frames(q)[-1]

    def q_from_joint_states(self, positions: Dict[str, float]) -> List[float]:
        return [positions.get(j, 0.0) for j in self.joints]


@dataclass
class HandKinematics:
    base_frame: str
    fingers: Dict[str, FingerChain]

    @classmethod
    def from_yaml(cls, path: str) -> 'HandKinematics':
        with open(path) as f:
            cfg = yaml.safe_load(f)
        fingers = {}
        for name, fc in cfg['fingers'].items():
            rows = [DHRow(*map(float, row)) for row in fc['dh']]
            base = np.array(fc['base_transform'], dtype=float)
            if base.shape != (4, 4):
                raise ValueError(f'{name}: base_transform must be 4x4')
            if len(rows) != len(fc['joints']):
                raise ValueError(f'{name}: number of joints and DH rows differ')
            fingers[name] = FingerChain(name, list(fc['joints']), rows, base, fc['tip_frame'])
        return cls(cfg.get('base_frame', 'palm_link'), fingers)


def format_matrix(m: np.ndarray, translation_scale: float = 1000.0, digits: int = 3) -> str:
    """Pretty print a homogeneous matrix (translation in mm by default)."""
    lines = []
    for r in range(4):
        vals = [m[r, c] * (translation_scale if (c == 3 and r < 3) else 1.0) for c in range(4)]
        lines.append('| ' + ' '.join(f'{v:9.{digits}f}' for v in vals) + ' |')
    return '\n'.join(lines)
