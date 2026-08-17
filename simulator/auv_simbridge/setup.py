from glob import glob

from setuptools import find_packages, setup
import os

package_name = 'auv_simbridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhruv',
    maintainer_email='dhruvsavot01@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simulator_bridge = auv_simbridge.simulator_bridge:main',
            'teleop = auv_simbridge.teleop:main',
            'simulator_node = auv_simbridge.simulator_node:main',
            'vision_test = auv_simbridge.vision_test:main',
            'ocean_current = auv_simbridge.ocean_current:main',
            'mini_simulator_bridge = auv_simbridge.mini_simulator_bridge:main',
        ],
    },
)
