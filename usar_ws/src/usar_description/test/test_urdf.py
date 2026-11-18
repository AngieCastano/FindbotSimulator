"""Validación del modelo del robot: el xacro produce un URDF correcto y coherente."""

import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO_FILE = os.path.join(PKG_DIR, 'urdf', 'findbot.urdf.xacro')
PARAMS_FILE = os.path.join(PKG_DIR, 'config', 'robot.yaml')
# Radio con el que usar_worldgen infla obstáculos (config/default_params.yaml).
WORLDGEN_ROBOT_RADIUS = 0.30


@pytest.fixture(scope='module')
def urdf_xml():
    doc = xacro.process_file(XACRO_FILE, mappings={'robot_params': PARAMS_FILE})
    return doc.toxml()


@pytest.fixture(scope='module')
def params():
    with open(PARAMS_FILE, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_frames_and_joints(urdf_xml):
    root = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in root.findall('link')}
    assert links == {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
                     'caster_link', 'lidar_link', 'camera_link', 'camera_optical_frame'}
    joints = {j.get('name'): j.get('type') for j in root.findall('joint')}
    assert joints['left_wheel_joint'] == joints['right_wheel_joint'] == 'continuous'


def test_sensors_and_plugins(urdf_xml):
    root = ET.fromstring(urdf_xml)
    sensors = {s.get('name'): s for s in root.iter('sensor')}
    assert sensors['lidar'].get('type') == 'gpu_lidar'
    assert sensors['lidar'].findtext('topic') == '/scan'
    assert sensors['lidar'].find('lidar/noise/type').text == 'gaussian'
    assert sensors['rgbd_camera'].get('type') == 'rgbd_camera'
    assert sensors['rgbd_camera'].findtext('gz_frame_id') == 'camera_optical_frame'
    plugins = {p.get('name') for p in root.iter('plugin')}
    assert 'gz::sim::systems::DiffDrive' in plugins
    assert 'gz::sim::systems::JointStatePublisher' in plugins


def test_footprint_fits_worldgen_radius(params):
    """La huella del robot cabe en el radio usado para el área explorable."""
    ch, wh = params['chassis'], params['wheels']
    # Distancias medidas desde base_footprint (centro de giro, sobre el eje de ruedas).
    corners = math.hypot(abs(ch['x_offset']) + ch['length'] / 2, ch['width'] / 2)
    wheel = wh['separation'] / 2 + wh['radius']  # colisión esférica de la rueda
    assert max(corners, wheel) <= WORLDGEN_ROBOT_RADIUS


def test_lidar_plane_below_min_obstacle_height(params):
    """El plano del LiDAR queda por debajo de la altura mínima de escombros (0.4 m)."""
    height = params['wheels']['radius'] + params['lidar']['z']
    assert 0.1 < height < 0.4


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf no instalado')
def test_check_urdf(urdf_xml, tmp_path):
    path = tmp_path / 'findbot.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(shutil.which('gz') is None, reason='Gazebo no instalado')
def test_converts_to_sdf(urdf_xml, tmp_path):
    path = tmp_path / 'findbot.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    model = ET.fromstring(result.stdout).find('model')
    sensors = {s.get('name') for s in model.iter('sensor')}
    assert {'lidar', 'rgbd_camera'} <= sensors
