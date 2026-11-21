"""
Regla crítica: el ground truth NO puede llegar al robot.

Pruebas estáticas sobre el código fuente de todo el workspace:
- Ningún paquete del robot importa usar_worldgen.ground_truth ni usa 'ground_truth' en código
  (se permiten comentarios y docstrings que recuerdan la regla).
- Solo usar_metrics (evaluador), las herramientas offline de usar_worldgen y mission.launch.py
  (que pasa la ruta al evaluador) lo mencionan en código.
- El evaluador no publica nada: no puede devolver información al robot.
- El puente ROS–Gazebo no expone poses de modelos (víctimas, escombros) y el robot no usa
  plugins de Gazebo que publiquen su pose real.

Excepción (CLAUDE.md): los simuladores de sensores de la Capa 2 (usar_sim_sensors) pueden leer
el ground truth y la pose real (/sim/*). Se comprueba que:
- solo publican mensajes de medida de sensor (nunca poses, marcadores ni detecciones),
- ningún otro paquete del robot nombra un tópico /sim/* salvo la configuración del puente,
- la pose real solo existe con sim_pose:=true y va a /sim/*,
- sim.launch.py entrega la ruta del ground truth solo a nodos de usar_sim_sensors.
"""

import ast
import os

import pytest
import yaml

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# usar_sim_sensors NO está en la lista: es la excepción de la Capa 2 (ver docstring).
ROBOT_PACKAGES = ['usar_description', 'usar_bringup', 'usar_exploration', 'usar_detection']
SIM_SENSORS = 'usar_sim_sensors'
#: Mensajes que puede publicar un simulador de sensores: solo medidas.
MEASUREMENT_TYPES = {'Image', 'GasConcentration', 'SoundDirection'}
#: Archivos del robot autorizados a nombrar el ground truth, y por qué.
ALLOWED = {
    os.path.join('usar_bringup', 'launch', 'mission.launch.py'):
        'genera el mundo si falta y pasa la ruta del ground truth SOLO al evaluador',
    os.path.join('usar_bringup', 'launch', 'sim.launch.py'):
        'pasa la ruta del ground truth SOLO a los simuladores de sensores (usar_sim_sensors)',
}


def _python_files(package):
    root = os.path.join(SRC, package)
    for base, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in ('test', '__pycache__')]
        for name in files:
            if name.endswith('.py'):
                yield os.path.join(base, name)


def _docstring_nodes(tree):
    """Nodos Constant que son docstrings (módulo, clases, funciones)."""
    docs = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            body = node.body
            if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant):
                docs.add(id(body[0].value))
    return docs


def ground_truth_uses(path):
    """Usos del ground truth en el código (no en comentarios ni docstrings) de un archivo."""
    with open(path, 'r', encoding='utf-8') as stream:
        tree = ast.parse(stream.read(), filename=path)
    docs = _docstring_nodes(tree)
    uses = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            uses += [a.name for a in node.names if 'ground_truth' in a.name]
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ''
            if 'ground_truth' in module or any('ground_truth' in a.name for a in node.names):
                uses.append(f'from {module} import ...')
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            if 'ground_truth' in node.value and id(node) not in docs:
                uses.append(f'línea {node.lineno}: {node.value!r}')
        elif isinstance(node, ast.Name) and 'ground_truth' in node.id:
            uses.append(f'línea {node.lineno}: {node.id}')
        elif isinstance(node, ast.Attribute) and 'ground_truth' in node.attr:
            uses.append(f'línea {node.lineno}: .{node.attr}')
    return uses


@pytest.mark.parametrize('package', ROBOT_PACKAGES)
def test_robot_packages_do_not_use_ground_truth(package):
    offending = {}
    for path in _python_files(package):
        rel = os.path.relpath(path, SRC)
        if rel in ALLOWED:
            continue
        uses = ground_truth_uses(path)
        if uses:
            offending[rel] = uses
    assert not offending, f'Código del robot que usa el ground truth: {offending}'


def test_detector_finds_real_uses(tmp_path):
    """Comprueba que la prueba anterior detecta usos reales y no solo menciones."""
    bad = tmp_path / 'bad.py'
    bad.write_text('"""Nunca lee ground_truth.yaml."""\n'
                   '# ground_truth en un comentario\n'
                   'from usar_worldgen.ground_truth import load_ground_truth\n'
                   "PATH = 'worlds/seed1/ground_truth.yaml'\n")
    uses = ground_truth_uses(str(bad))
    assert len(uses) == 2
    good = tmp_path / 'good.py'
    good.write_text('"""Nunca lee ground_truth.yaml."""\n# ground_truth\nX = 1\n')
    assert ground_truth_uses(str(good)) == []


def test_allowed_launch_only_forwards_to_evaluator():
    """mission.launch.py solo entrega el ground truth al nodo evaluator."""
    path = os.path.join(SRC, 'usar_bringup', 'launch', 'mission.launch.py')
    with open(path, 'r', encoding='utf-8') as stream:
        tree = ast.parse(stream.read())
    receivers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
            kwargs = {k.arg: k.value for k in node.keywords}
            if 'ground_truth_file' in ast.dump(kwargs.get('parameters', ast.Constant(None))):
                receivers.add(kwargs['executable'].value)
    assert receivers == {'evaluator'}


def test_sim_launch_only_forwards_to_sim_sensors():
    """sim.launch.py solo entrega el ground truth a nodos de usar_sim_sensors."""
    path = os.path.join(SRC, 'usar_bringup', 'launch', 'sim.launch.py')
    with open(path, 'r', encoding='utf-8') as stream:
        tree = ast.parse(stream.read())
    packages = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
            kwargs = {k.arg: k.value for k in node.keywords}
            if 'ground_truth_file' in ast.dump(kwargs.get('parameters', ast.Constant(None))):
                packages.add(kwargs['package'].value)
    assert packages == {SIM_SENSORS}


def test_sim_sensors_publish_only_measurements():
    """Los simuladores de sensores publican medidas, nunca posiciones ni detecciones."""
    published = set()
    for path in _python_files(SIM_SENSORS):
        with open(path, 'r', encoding='utf-8') as stream:
            tree = ast.parse(stream.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == \
                    'create_publisher':
                msg_type = node.args[0]
                published.add(getattr(msg_type, 'id', None) or ast.dump(msg_type))
    assert published, 'usar_sim_sensors debería publicar alguna medida'
    assert published <= MEASUREMENT_TYPES, \
        f'Mensajes no permitidos: {published - MEASUREMENT_TYPES}'


@pytest.mark.parametrize('package', ['usar_bringup', 'usar_exploration', 'usar_detection',
                                     'usar_metrics'])
def test_robot_does_not_subscribe_to_sim_topics(package):
    """Los tópicos /sim/* (pose real, imagen cruda) solo los usan los simuladores de sensores."""
    offending = []
    root = os.path.join(SRC, package)
    for base, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in ('test', '__pycache__')]
        for name in files:
            if name.endswith(('.py', '.yaml')):
                path = os.path.join(base, name)
                with open(path, 'r', encoding='utf-8') as stream:
                    text = stream.read()
                if "'/sim/" in text or ' /sim/' in text:
                    offending.append(os.path.relpath(path, SRC))
    assert not offending, f'Archivos del robot que nombran /sim/*: {offending}'


def test_evaluator_publishes_nothing():
    for path in _python_files('usar_metrics'):
        with open(path, 'r', encoding='utf-8') as stream:
            tree = ast.parse(stream.read())
        calls = [n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        assert 'create_publisher' not in calls, f'{path} publica en ROS'


@pytest.mark.parametrize('bridge_file', ['bridge.yaml', 'bridge_thermal.yaml',
                                         'bridge_sim_pose.yaml'])
def test_bridge_does_not_expose_model_poses(bridge_file):
    path = os.path.join(SRC, 'usar_description', 'config', bridge_file)
    with open(path, 'r', encoding='utf-8') as stream:
        entries = yaml.safe_load(stream)
    # La imagen térmica es una medida del sensor (temperatura por píxel), no una posición.
    allowed = {'/clock', '/cmd_vel', '/odom', '/tf', '/joint_states', '/scan', '/camera/image',
               '/camera/depth_image', '/camera/camera_info', '/sim/thermal/image_raw',
               '/sim/ground_truth/odom'}  # pose real: solo para usar_sim_sensors
    topics = {e['gz_topic_name'] for e in entries}
    assert topics <= allowed, f'Tópicos de Gazebo no auditados en el puente: {topics - allowed}'
    for entry in entries:
        assert '/world/' not in entry['gz_topic_name']
        assert '/model/' not in entry['gz_topic_name']


def _robot_urdf(**mappings):
    import xacro
    path = os.path.join(SRC, 'usar_description', 'urdf', 'findbot.urdf.xacro')
    params = os.path.join(SRC, 'usar_description', 'config', 'robot.yaml')
    return xacro.process_file(path, mappings={'robot_params': params, **mappings}).toxml()


def test_robot_odometry_comes_from_wheels():
    """La odometría sale de DiffDrive (cinemática de ruedas), no de la pose real de Gazebo."""
    text = _robot_urdf()
    assert 'gz-sim-diff-drive-system' in text
    for plugin in ('odometry-publisher', 'pose-publisher'):
        assert plugin not in text, f'Plugin que publica la pose real: {plugin}'


def test_real_pose_only_for_sim_sensors():
    """Con sim_pose:=true la pose real va a /sim/* (solo la leen los simuladores)."""
    text = _robot_urdf(sim_pose='true')
    assert 'odometry-publisher' in text
    assert '<odom_topic>/sim/ground_truth/odom</odom_topic>' in text
    assert 'pose-publisher' not in text
