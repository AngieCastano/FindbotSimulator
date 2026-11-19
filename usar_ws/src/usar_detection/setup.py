from glob import glob

from setuptools import find_packages, setup

package_name = 'usar_detection'

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
    description='Detector de víctimas simuladas por color y profundidad (victim_detector)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'victim_detector = usar_detection.victim_detector:main',
        ],
    },
)
