from glob import glob

from setuptools import find_packages, setup

package_name = 'usar_sim_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AngieCastano',
    maintainer_email='AngieCastano@todo.todo',
    description='Simuladores de sensores de la Capa 2 que Gazebo no modela por completo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thermal_camera_sim = usar_sim_sensors.thermal_camera_sim:main',
            'co2_sim = usar_sim_sensors.co2_sim:main',
            'sound_sim = usar_sim_sensors.sound_sim:main',
            'odom_noise_sim = usar_sim_sensors.odom_noise_sim:main',
        ],
    },
)
