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
# Radio con el que usar_worldgen infla obstáculos (= robot_radius de Nav2).
WORLDGEN_PARAMS = os.path.join(os.path.dirname(PKG_DIR), 'usar_worldgen', 'config',
                               'default_params.yaml')


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
                     'caster_link', 'lidar_link', 'imu_link', 'camera_link',
                     'camera_optical_frame'}
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
    assert sensors['imu'].get('type') == 'imu'
    assert sensors['imu'].findtext('topic') == '/imu/data_raw'
    assert sensors['imu'].find('imu/angular_velocity/z/noise/stddev') is not None
    plugins = {p.get('name') for p in root.iter('plugin')}
    assert 'gz::sim::systems::DiffDrive' in plugins
    assert 'gz::sim::systems::JointStatePublisher' in plugins
    assert 'gz::sim::systems::Imu' in plugins


def test_footprint_fits_worldgen_radius(params):
    """La huella del robot cabe en el radio usado para el área explorable."""
    ch, wh = params['chassis'], params['wheels']
    # Distancias medidas desde base_footprint (centro de giro, sobre el eje de ruedas).
    corners = math.hypot(abs(ch['x_offset']) + ch['length'] / 2, ch['width'] / 2)
    wheel = wh['separation'] / 2 + wh['radius']  # colisión esférica de la rueda
    with open(WORLDGEN_PARAMS, encoding='utf-8') as stream:
        radius = yaml.safe_load(stream)['robot_radius']
    assert max(corners, wheel) <= radius
    assert radius - max(corners, wheel) == pytest.approx(0.05, abs=0.01)   # margen de 5 cm


def test_geometry_matches_hardware(params):
    """Posiciones y medidas de hardware/CLAUDE.md (sección Geometría), respecto a base_link."""
    ch, wh, cs = params['chassis'], params['wheels'], params['caster']
    assert (ch['length'], ch['width'], ch['height']) == pytest.approx((0.22, 0.16, 0.066))
    assert (ch['x_offset'], ch['z_offset']) == pytest.approx((-0.05, 0.0455))
    assert (wh['radius'], wh['width'], wh['separation']) == pytest.approx((0.0325, 0.026, 0.196))
    assert (cs['radius'], cs['x_offset']) == pytest.approx((0.0125, -0.14))
    assert (params['lidar']['x'], params['lidar']['z']) == pytest.approx((-0.03, 0.1125))
    assert (params['camera']['x'], params['camera']['z']) == pytest.approx((0.055, 0.091))
    imu = params['imu']
    assert (imu['x'], imu['y'], imu['z']) == pytest.approx((0.0, 0.0, 0.0255))
    rad = params['radar']
    assert (rad['x'], rad['y'], rad['z']) == pytest.approx((0.06, 0.0, 0.037))
    dd = params['diff_drive']
    assert dd['max_linear_velocity'] <= 0.30 and dd['max_linear_acceleration'] <= 0.5
    # Nada por encima del plano del LiDAR (0.145 m del suelo).
    assert wh['radius'] + params['lidar']['z'] == pytest.approx(0.145)


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


# ---------------------------------------------------------------------- cámara térmica (fase 2)
@pytest.fixture(scope='module')
def thermal_xml():
    doc = xacro.process_file(XACRO_FILE, mappings={'robot_params': PARAMS_FILE,
                                                   'use_thermal': 'true'})
    return doc.toxml()


def test_thermal_off_by_default(urdf_xml):
    root = ET.fromstring(urdf_xml)
    assert 'thermal' not in {s.get('type') for s in root.iter('sensor')}
    assert 'thermal_link' not in {link.get('name') for link in root.findall('link')}


def test_thermal_camera(thermal_xml, params):
    root = ET.fromstring(thermal_xml)
    links = {link.get('name') for link in root.findall('link')}
    assert {'thermal_link', 'thermal_optical_frame'} <= links
    sensor = next(s for s in root.iter('sensor') if s.get('type') == 'thermal')
    assert sensor.findtext('topic') == '/sim/thermal/image_raw'
    assert sensor.findtext('gz_frame_id') == 'thermal_optical_frame'
    assert sensor.find('camera/noise') is None          # Gazebo falla con <noise> en la térmica
    image = sensor.find('camera/image')
    th = params['thermal']
    assert (int(image.findtext('width')), int(image.findtext('height'))) == \
        (th['width'], th['height']) == (32, 24)
    assert image.findtext('format') == 'L16'
    assert math.degrees(float(sensor.findtext('camera/horizontal_fov'))) == \
        pytest.approx(55.0, abs=0.5)
    # Relativa a la cámara RGB-D como en el robot físico: 6 cm a la derecha, 3 mm más abajo.
    cam = params['camera']
    assert (th['x'], th['y'], th['z']) == pytest.approx((cam['x'], -0.06, cam['z'] - 0.003))


def test_thermal_check_urdf(thermal_xml, tmp_path):
    path = tmp_path / 'findbot_thermal.urdf'
    path.write_text(thermal_xml)
    result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


# ---------------------------------------------------------------------- sensor de CO₂ (fase 2)
def test_co2_link(params):
    off = ET.fromstring(xacro.process_file(XACRO_FILE, mappings={
        'robot_params': PARAMS_FILE}).toxml())
    assert 'co2_link' not in {link.get('name') for link in off.findall('link')}
    root = ET.fromstring(xacro.process_file(XACRO_FILE, mappings={
        'robot_params': PARAMS_FILE, 'use_co2': 'true'}).toxml())
    joint = next(j for j in root.findall('joint') if j.get('name') == 'co2_joint')
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    co2 = params['co2']
    assert xyz == pytest.approx([co2['x'], co2['y'], co2['z']])
    # Posición tomada de hardware/CLAUDE.md (Geometría: TGS4161 en co2_link).
    assert xyz == pytest.approx([0.03, 0.05, 0.0275])


# ---------------------------------------------------------------------- micrófonos (fase 2)
def test_mic_array_link(params):
    root = ET.fromstring(xacro.process_file(XACRO_FILE, mappings={
        'robot_params': PARAMS_FILE, 'use_sound': 'true'}).toxml())
    joint = next(j for j in root.findall('joint') if j.get('name') == 'mic_array_joint')
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    # Posición tomada de hardware/CLAUDE.md (Geometría: ReSpeaker en mic_array_link).
    assert xyz == pytest.approx([-0.12, 0.0, 0.0835])
    mic = params['mic_array']
    assert xyz == pytest.approx([mic['x'], mic['y'], mic['z']])
