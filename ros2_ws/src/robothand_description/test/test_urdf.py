"""Checks the URDF model: xacro expansion, kinematic tree, meshes, DH file."""

import os

import pytest
import urdf_parser_py.urdf as urdf
import xacro
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO_FILE = os.path.join(PKG_DIR, 'urdf', 'robothand.urdf.xacro')
DH_FILE = os.path.join(PKG_DIR, 'config', 'dh_parameters.yaml')

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']
ACTUATED = [f + '_mcp_joint' for f in FINGERS]


@pytest.fixture(scope='module')
def robot():
    xml = xacro.process_file(XACRO_FILE).toxml()
    return urdf.URDF.from_xml_string(xml)


def test_root_and_tips(robot):
    assert robot.get_root() == 'base_link'
    links = {link.name for link in robot.links}
    for finger in FINGERS:
        assert finger + '_tip' in links


def test_actuated_joints_are_not_mimic(robot):
    joints = {j.name: j for j in robot.joints}
    for name in ACTUATED:
        assert joints[name].type == 'revolute'
        assert joints[name].mimic is None


def test_mimic_joints_reference_finger_mcp(robot):
    for joint in robot.joints:
        if joint.mimic is not None:
            finger = joint.name.split('_')[0]
            assert joint.mimic.joint == finger + '_mcp_joint'


def test_meshes_exist(robot):
    prefix = 'package://robothand_description/'
    for link in robot.links:
        for visual in link.visuals:
            filename = visual.geometry.filename
            assert filename.startswith(prefix)
            assert os.path.isfile(os.path.join(PKG_DIR, filename[len(prefix):])), filename


def test_dh_file_matches_urdf(robot):
    with open(DH_FILE) as f:
        dh = yaml.safe_load(f)
    joints = {j.name for j in robot.joints}
    links = {link.name for link in robot.links}
    assert dh['base_frame'] in links
    assert set(dh['fingers']) == set(FINGERS)
    for finger in dh['fingers'].values():
        assert finger['tip_frame'] in links
        assert len(finger['joints']) == len(finger['dh'])
        for name in finger['joints']:
            assert name in joints
        assert len(finger['base_transform']) == 4
