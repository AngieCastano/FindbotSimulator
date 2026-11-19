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
"""

import ast
import os

import pytest
import yaml

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROBOT_PACKAGES = ['usar_description', 'usar_bringup', 'usar_exploration', 'usar_detection']
#: Archivos del robot autorizados a nombrar el ground truth, y por qué.
ALLOWED = {
    os.path.join('usar_bringup', 'launch', 'mission.launch.py'):
        'genera el mundo si falta y pasa la ruta del ground truth SOLO al evaluador',
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


def test_evaluator_publishes_nothing():
    for path in _python_files('usar_metrics'):
        with open(path, 'r', encoding='utf-8') as stream:
            tree = ast.parse(stream.read())
        calls = [n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        assert 'create_publisher' not in calls, f'{path} publica en ROS'


def test_bridge_does_not_expose_model_poses():
    path = os.path.join(SRC, 'usar_description', 'config', 'bridge.yaml')
    with open(path, 'r', encoding='utf-8') as stream:
        entries = yaml.safe_load(stream)
    allowed = {'/clock', '/cmd_vel', '/odom', '/tf', '/joint_states', '/scan', '/camera/image',
               '/camera/depth_image', '/camera/camera_info'}
    topics = {e['gz_topic_name'] for e in entries}
    assert topics <= allowed, f'Tópicos de Gazebo no auditados en el puente: {topics - allowed}'
    for entry in entries:
        assert '/world/' not in entry['gz_topic_name']
        assert '/model/' not in entry['gz_topic_name']


def test_robot_odometry_comes_from_wheels():
    """La odometría sale de DiffDrive (cinemática de ruedas), no de la pose real de Gazebo."""
    urdf_dir = os.path.join(SRC, 'usar_description', 'urdf')
    text = ''.join(open(os.path.join(urdf_dir, f), encoding='utf-8').read()
                   for f in os.listdir(urdf_dir) if f.endswith(('.xacro', '.urdf')))
    assert 'gz-sim-diff-drive-system' in text
    for plugin in ('odometry-publisher', 'pose-publisher'):
        assert plugin not in text, f'Plugin que publica la pose real: {plugin}'
