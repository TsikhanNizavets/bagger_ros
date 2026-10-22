import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bagger_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'textures'), glob('textures/*.png')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tsikhan Nizavets',
    maintainer_email='tsikhannizavets@gmail.com',
    description='Launches cube detection using AprilTags and AprilTag boards',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_debug_view = bagger_perception.camera_debug_view:main',
            'cube_tracker = bagger_perception.cube_tracker:main',
        ],
    },
)
