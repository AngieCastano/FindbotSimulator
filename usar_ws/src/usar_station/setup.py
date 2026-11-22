from glob import glob

from setuptools import find_packages, setup

package_name = 'usar_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AngieCastano',
    maintainer_email='AngieCastano@todo.todo',
    description='Estación remota de teleoperación y enlace robot ↔ estación',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'comm_relay = usar_station.comm_relay:main',
            'link_failsafe = usar_station.link_failsafe:main',
            'sim_operator = usar_station.sim_operator:main',
        ],
    },
)
