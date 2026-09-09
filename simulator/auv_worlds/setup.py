from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_worlds'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # camera_director, run by competition.launch.py. A plain script in
        # share/ rather than a console_script on purpose: the demo overlay
        # already mounts this directory, so it can be edited and picked up by a
        # restart, while a new console_script needs entry-point metadata that
        # only an image rebuild writes.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'media', 'textures'),
         glob('media/textures/*')),
        (os.path.join('share', package_name, 'models', 'auv_test'),
         ['models/auv_test/model.config', 'models/auv_test/model.sdf']),
        (os.path.join('share', package_name, 'models', 'auv_thrusters'),
         ['models/auv_thrusters/model.config', 'models/auv_thrusters/model.sdf']),
        (os.path.join('share', package_name, 'models', 'm7urdfnew_sdf_package'),
         ['models/m7urdfnew_sdf_package/model.config',
          'models/m7urdfnew_sdf_package/model.sdf']),
        (os.path.join('share', package_name, 'models', 'm7urdfnew_sdf_package', 'meshes'),
         glob('models/m7urdfnew_sdf_package/meshes/*')),
        (os.path.join('share', package_name, 'models', 'new_assem_without_battery_hull', 'meshes'),
         glob('models/new_assem_without_battery_hull/meshes/*')),
     
        (os.path.join('share', package_name, 'models', 'rs2026_gate'),
         ['models/rs2026_gate/model.config', 'models/rs2026_gate/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_gate', 'meshes'),
         glob('models/rs2026_gate/meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'rs2026_bin_urdf'),
         ['models/rs2026_bin_urdf/model.config', 'models/rs2026_bin_urdf/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_bin_urdf', 'meshes'),
         glob('models/rs2026_bin_urdf/meshes/*')),
        (os.path.join('share', package_name, 'models', 'slalom_red'),
         ['models/slalom_red/model.config', 'models/slalom_red/model.sdf']),
        (os.path.join('share', package_name, 'models', 'slalom_red', 'meshes'),
         glob('models/slalom_red/meshes/*')),
        (os.path.join('share', package_name, 'models', 'slalom_white'),
         ['models/slalom_white/model.config', 'models/slalom_white/model.sdf']),
        (os.path.join('share', package_name, 'models', 'slalom_white', 'meshes'),
         glob('models/slalom_white/meshes/*')),
        (os.path.join('share', package_name, 'models', 'rs2026_torpedo'),
         ['models/rs2026_torpedo/model.config', 'models/rs2026_torpedo/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_torpedo', 'meshes'),
         glob('models/rs2026_torpedo/meshes/*')),
        (os.path.join('share', package_name, 'models', 'torPEDO'),
         ['models/torPEDO/model.config', 'models/torPEDO/model.sdf']),
        (os.path.join('share', package_name, 'models', 'torPEDO', 'meshes'),
         glob('models/torPEDO/meshes/*')),
        (os.path.join('share', package_name, 'models', 'marker'),
         ['models/marker/model.config', 'models/marker/model.sdf']),
        (os.path.join('share', package_name, 'models', 'marker', 'meshes'),
         glob('models/marker/meshes/*')),
        (os.path.join('share', package_name, 'models', 'rs2026_octogon'),
         ['models/rs2026_octogon/model.config', 'models/rs2026_octogon/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_octogon', 'meshes'),
         glob('models/rs2026_octogon/meshes/*')),
        # Graspable Task 5 props. Their visual meshes live in rs2026_octogon/meshes,
        # which is installed above, so only the two XML files ship for each.
        (os.path.join('share', package_name, 'models', 'rs2026_pickup_bandaid'),
         ['models/rs2026_pickup_bandaid/model.config', 'models/rs2026_pickup_bandaid/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_pickup_plug'),
         ['models/rs2026_pickup_plug/model.config', 'models/rs2026_pickup_plug/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_pickup_capsule'),
         ['models/rs2026_pickup_capsule/model.config', 'models/rs2026_pickup_capsule/model.sdf']),
        (os.path.join('share', package_name, 'models', 'rs2026_pickup_screw'),
         ['models/rs2026_pickup_screw/model.config', 'models/rs2026_pickup_screw/model.sdf']),
        (os.path.join('share', package_name, 'models', 'path_marker'),
         ['models/path_marker/model.config', 'models/path_marker/model.sdf']),
        (os.path.join('share', package_name, 'models', 'path_marker', 'meshes'),
         glob('models/path_marker/meshes/*')),
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
            'generate_world = auv_worlds.world_generator:main',
            'mesh_limits = auv_worlds.mesh_limits:main',
        ],
    },
)
