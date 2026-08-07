import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bugger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install all launch files from the launch/ folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install all URDF files from the urdf/ folder
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # 3. (Optional) Install mesh or world files if you have them
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TsikhanNizavets',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_bugger = bugger.teleop_bugger:main',
            'container_unloader = bugger.container_unloader:main',
        ],
    },
)
