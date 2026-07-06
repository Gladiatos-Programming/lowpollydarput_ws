from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'darput_description'

setup(
    name=package_name,         
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'darput_description'), glob('darput_description/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dar',
    maintainer_email='dar@todo.todo',
    description='Controller test',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Init_Position = darput_description.Init_Position:main',
            'imu_reader = darput_description.imu_reader:main',
            'VisualMarker = darput_description.VisualMarker:main',
            'PinnochioIK = darput_description.PinnochioIK:main',
            'Pre_Walking_state = darput_description.Pre_Walking_state:main',
            'Walking_State = darput_description.Walking_State:main',
            'UI_Interface = darput_description.UI_Interface:main',
            'foot_sensor = darput_description.foot_sensor:main',
            'test_foot_state = darput_description.test_foot_state:main',
        ],
    },
)
