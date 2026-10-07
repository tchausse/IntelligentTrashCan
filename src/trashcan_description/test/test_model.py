"""Checks of the expanded robot model."""

import math
import pathlib
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
CONTROLLERS = PACKAGE / 'config' / 'trashcan_controllers.yaml'


@pytest.fixture(scope='module')
def robot():
    urdf = xacro.process_file(
        str(PACKAGE / 'urdf' / 'trashcan.urdf.xacro'),
        mappings={'controllers_file': str(CONTROLLERS)},
    ).toxml()
    return ET.fromstring(urdf)


@pytest.fixture(scope='module')
def drive_params():
    with open(CONTROLLERS) as f:
        return yaml.safe_load(f)['diff_drive_controller']['ros__parameters']


def joint(robot, name):
    element = robot.find(f"joint[@name='{name}']")
    assert element is not None, f'missing joint {name}'
    return element


def xyz(element):
    origin = element.find('origin')
    return [float(v) for v in origin.get('xyz', '0 0 0').split()]


def test_every_link_with_geometry_has_valid_inertia(robot):
    for link in robot.findall('link'):
        if link.find('visual') is None and link.find('collision') is None:
            continue
        inertial = link.find('inertial')
        assert inertial is not None, f'{link.get("name")} has no inertial'
        assert float(inertial.find('mass').get('value')) > 0
        i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        principal = sorted([i['ixx'], i['iyy'], i['izz']])
        assert principal[0] > 0, f'{link.get("name")} has a non-positive inertia'
        # Triangle inequality of principal moments of inertia.
        assert principal[0] + principal[1] >= principal[2] * (1 - 1e-9), link.get('name')


def test_total_mass_is_plausible(robot):
    mass = sum(float(m.get('value')) for m in robot.iter('mass'))
    assert 8.0 < mass < 12.0


def test_controller_matches_wheel_geometry(robot, drive_params):
    left = xyz(joint(robot, 'left_wheel_joint'))
    right = xyz(joint(robot, 'right_wheel_joint'))
    assert math.isclose(left[1] - right[1], drive_params['wheel_separation'])

    wheel = robot.find("link[@name='left_wheel_link']/collision/geometry/cylinder")
    assert math.isclose(float(wheel.get('radius')), drive_params['wheel_radius'])
    # base_link rides on the wheel axle.
    assert math.isclose(xyz(joint(robot, 'base_joint'))[2], drive_params['wheel_radius'])


def test_bin_is_open_on_top(robot):
    bin_wall = robot.find("link[@name='bin_wall_link']")
    assert bin_wall.find('collision/geometry/cylinder') is None
    assert len(bin_wall.findall('collision/geometry/box')) >= 16


def test_drive_wheels_are_controlled_by_ros2_control(robot):
    joints = {j.get('name') for j in robot.findall('ros2_control/joint')}
    assert joints == {'left_wheel_joint', 'right_wheel_joint'}
    plugin = robot.find('gazebo/plugin')
    assert plugin.get('filename') == 'gz_ros2_control-system'
    assert plugin.find('parameters').text == str(CONTROLLERS)


@pytest.mark.parametrize('sensor, sensor_type, topic', [
    ('lidar', 'gpu_lidar', 'scan'),
    ('camera', 'rgbd_camera', 'camera'),
    ('imu', 'imu', 'imu'),
    ('fill_sensor', 'gpu_lidar', 'bin/fill_scan'),
])
def test_sensors_are_declared(robot, sensor, sensor_type, topic):
    element = robot.find(f"gazebo/sensor[@name='{sensor}']")
    assert element is not None, f'missing sensor {sensor}'
    assert element.get('type') == sensor_type
    assert element.find('topic').text == topic
    frame = element.find('gz_frame_id').text
    assert robot.find(f"link[@name='{frame}']") is not None
