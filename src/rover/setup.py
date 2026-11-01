from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

	(
	    os.path.join('share', package_name, 'config'),
	    glob('config/*')
	),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Tanay Thapar',
    maintainer_email='tanay.ggmu@gmail.com',

    description='All Weather Rover simulation with Ackermann steering, Gazebo and ros2_control',

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'fusion_node = rover.nodes.fusion_node:main',
            'data_recorder_node = rover.nodes.data_recorder_node:main',
            'train = rover.scripts.train:main',
            'evaluate = rover.scripts.evaluate:main',
            'prepare_personal_data = rover.scripts.prepare_personal_data:main',
        ],
    },
)
