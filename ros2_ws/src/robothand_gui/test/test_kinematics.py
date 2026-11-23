"""The DH parameters must describe the same kinematics as the URDF model.

For random joint angles the following is checked for every finger:
  * DH tip position == URDF tip frame position
  * origin of DH frame i (i = 1..n-1) lies on the axis of URDF joint i+1
  * z axis of DH frame i-1 == axis of URDF joint i (same sign!)
"""

import math
import os

import numpy as np
import pytest
from robothand_gui.kinematics import dh_matrix, HandKinematics

HERE = os.path.dirname(os.path.abspath(__file__))
WS_SRC = os.path.dirname(os.path.dirname(HERE))
DESC = os.path.join(WS_SRC, 'robothand_description')


def _files():
    xacro_file = os.path.join(DESC, 'urdf', 'robothand.urdf.xacro')
    dh_file = os.path.join(DESC, 'config', 'dh_parameters.yaml')
    if not os.path.isfile(xacro_file):
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('robothand_description')
        xacro_file = os.path.join(share, 'urdf', 'robothand.urdf.xacro')
        dh_file = os.path.join(share, 'config', 'dh_parameters.yaml')
    return xacro_file, dh_file


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


class UrdfFK:
    def __init__(self, xml):
        import urdf_parser_py.urdf as urdf
        self.robot = urdf.URDF.from_xml_string(xml)
        self.joints = {j.name: j for j in self.robot.joints}
        self.child_joint = {j.child: j for j in self.robot.joints}

    def chain(self, root, tip):
        out, link = [], tip
        while link != root:
            j = self.child_joint[link]
            out.append(j)
            link = j.parent
        return list(reversed(out))

    def value(self, name, q):
        j = self.joints[name]
        if j.mimic is not None:
            return j.mimic.multiplier * self.value(j.mimic.joint, q) + (j.mimic.offset or 0.0)
        return q.get(name, 0.0)

    def joint_frames(self, root, tip, q):
        """World pose of every joint frame (before its rotation) and the tip."""
        m = np.eye(4)
        frames = {}
        for j in self.chain(root, tip):
            o = np.eye(4)
            if j.origin is not None:
                o[:3, :3] = _rpy(*(j.origin.rpy or [0, 0, 0]))
                o[:3, 3] = j.origin.xyz or [0, 0, 0]
            m = m @ o
            frames[j.name] = m.copy()
            if j.type != 'fixed':
                r = np.eye(4)
                r[:3, :3] = _axis_angle(j.axis, self.value(j.name, q))
                m = m @ r
        return frames, m


@pytest.fixture(scope='module')
def models():
    import xacro
    xacro_file, dh_file = _files()
    return UrdfFK(xacro.process_file(xacro_file).toxml()), HandKinematics.from_yaml(dh_file)


def test_dh_matrix_identity():
    assert np.allclose(dh_matrix(0, 0, 0, 0), np.eye(4))
    a = dh_matrix(math.pi / 2, 0.1, 0.2, -math.pi / 2)
    assert np.allclose(a[:3, 3], [0.0, 0.2, 0.1])
    assert np.allclose(a[:3, :3] @ a[:3, :3].T, np.eye(3))


@pytest.mark.parametrize('finger', ['thumb', 'index', 'middle', 'ring', 'little'])
def test_dh_matches_urdf(models, finger):
    urdf_fk, kin = models
    chain = kin.fingers[finger]
    rng = np.random.default_rng(42)
    independent = [n for n in chain.joints if urdf_fk.joints[n].mimic is None]
    for _ in range(50):
        q = {}
        for name in independent:
            lim = urdf_fk.joints[name].limit
            q[name] = rng.uniform(lim.lower, lim.upper)
        q_full = {n: urdf_fk.value(n, q) for n in chain.joints}
        frames_urdf, tip_urdf = urdf_fk.joint_frames(kin.base_frame, chain.tip_frame, q)
        frames_dh = chain.frames([q_full[n] for n in chain.joints])

        # tip position
        assert np.allclose(frames_dh[-1][:3, 3], tip_urdf[:3, 3], atol=1e-6), finger

        for i, name in enumerate(chain.joints):
            jf = frames_urdf[name]
            axis_world = jf[:3, :3] @ np.asarray(urdf_fk.joints[name].axis, float)
            # z_{i} of DH frame i (0-based: frame before joint i+1) == joint axis
            assert np.allclose(frames_dh[i][:3, 2], axis_world, atol=1e-6), (finger, name)
            # the DH origin lies on that joint axis
            d = frames_dh[i][:3, 3] - jf[:3, 3]
            assert np.linalg.norm(np.cross(d, axis_world)) < 1e-6, (finger, name)
