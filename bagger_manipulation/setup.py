import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bagger_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tsikhan Nizavets',
    maintainer_email='tsikhannizavets@gmail.com',
    description='Implements container manipulation for the bagger robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'container_unloader = bagger_manipulation.container_unloader:main',
            'dumping_executor = bagger_manipulation.dumping_executor:main',
        ],
    },
)
