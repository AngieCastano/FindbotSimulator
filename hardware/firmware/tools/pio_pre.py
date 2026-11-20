"""Script previo de PlatformIO: regenera include/config_gen.h antes de cada compilación.

Se ejecuta con el Python del sistema (que tiene PyYAML), no con el de PlatformIO.
"""
import shutil
import subprocess

Import('env')  # noqa: F821  (lo inyecta PlatformIO/SCons)

project_dir = env.subst('$PROJECT_DIR')  # noqa: F821
python = shutil.which('python3') or 'python3'
subprocess.check_call([python, f'{project_dir}/tools/gen_config.py'])
