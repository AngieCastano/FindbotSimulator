"""Los launch files se pueden importar y construir su LaunchDescription."""

import importlib.util
import os

import pytest

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch')


@pytest.mark.parametrize('name', ['sim.launch.py', 'nav.launch.py', 'exploration.launch.py'])
def test_launch_description_builds(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'),
                                                  os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    arguments = {a.name for a in description.get_launch_arguments()}
    assert {'seed', 'gui', 'rviz', 'worlds_dir'} <= arguments
