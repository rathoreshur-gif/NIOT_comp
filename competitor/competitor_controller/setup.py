import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'competitor_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsya',
    maintainer_email='sparshbadjate27@gmail.com',
    description='Reference controller for the Matsya AUV competition.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_controller = competitor_controller.basic_controller:main',
            # Demo-only: gamepad teleop. Separate entry point and separate
            # launch file, so the competition controller above is untouched.
            'gamepad_controller = competitor_controller.gamepad_controller:main',
        ],
    },
)
