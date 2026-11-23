import math

import pytest
from robothand_interface.hand_model import ActuatorMap, JointModel

URDF = """<robot name="t">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
    <limit lower="0" upper="1.5" effort="1" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="c"/>
    <limit lower="0" upper="1.5" effort="1" velocity="1"/>
    <mimic joint="j1" multiplier="0.5" offset="0.1"/></joint>
  <joint name="j3" type="revolute"><parent link="c"/><child link="d"/>
    <limit lower="0" upper="1.5" effort="1" velocity="1"/>
    <mimic joint="j2" multiplier="2.0"/></joint>
  <joint name="fixed" type="fixed"><parent link="a"/><child link="d"/></joint>
</robot>"""


def make_map():
    return ActuatorMap(['j1', 'k1'], [10.0, 170.0], [170.0, 10.0], [1.5, 1.0])


def test_closure_servo_roundtrip():
    m = make_map()
    assert m.closure_to_servo(0, 0.0) == 10.0
    assert m.closure_to_servo(0, 1.0) == 170.0
    assert m.closure_to_servo(1, 1.0) == 10.0           # inverted servo
    assert m.closure_to_servo(0, 2.0) == 170.0          # clamped
    for c in (0.0, 0.25, 0.5, 1.0):
        assert m.servo_to_closure(1, m.closure_to_servo(1, c)) == pytest.approx(c)


def test_joint_closure():
    m = make_map()
    assert m.joint_to_closure(0, 0.75) == pytest.approx(0.5)
    assert m.closure_to_joint(0, 0.5) == pytest.approx(0.75)
    assert m.joint_to_closure(0, -1.0) == 0.0
    assert m.index('k1') == 1 and m.index('nope') is None


def test_invalid_map():
    with pytest.raises(ValueError):
        ActuatorMap(['a'], [10.0], [10.0], [1.0])
    with pytest.raises(ValueError):
        ActuatorMap(['a', 'b'], [10.0], [20.0], [1.0])


def test_mimic_expansion_and_chain():
    jm = JointModel.from_urdf(URDF)
    assert jm.order == ['j1', 'j2', 'j3']
    assert jm.is_mimic('j2') and not jm.is_mimic('j1')
    names, pos = jm.expand({'j1': 1.0})
    vals = dict(zip(names, pos))
    assert vals['j1'] == 1.0
    assert vals['j2'] == pytest.approx(0.6)
    assert vals['j3'] == pytest.approx(1.2)
    assert jm.clamp('j1', 2.0) == 1.5


def test_empty_model_passthrough():
    names, pos = JointModel().expand({'x': 0.5})
    assert names == ['x'] and pos == [0.5]


def test_real_urdf_if_available():
    try:
        import xacro
        from ament_index_python.packages import get_package_share_directory
        path = get_package_share_directory('robothand_description') + '/urdf/robothand.urdf.xacro'
    except Exception:
        pytest.skip('robothand_description not available')
    jm = JointModel.from_urdf(xacro.process_file(path).toxml())
    names, pos = jm.expand({'index_mcp_joint': 1.0, 'thumb_mcp_joint': 0.5})
    vals = dict(zip(names, pos))
    assert len(names) == 15
    assert vals['index_pip_joint'] == pytest.approx(1.0)
    assert vals['index_dip_joint'] == pytest.approx(0.7)
    assert vals['thumb_ip_joint'] == pytest.approx(0.5)
    assert vals['middle_dip_joint'] == 0.0
    assert not math.isnan(sum(pos))
