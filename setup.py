import os, glob
from setuptools import setup

package_name = 'nml_task_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='nml@whit.contact',
    description='A ROS2 node for playing task-related audio cues.',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = nml_task_audio:main'
        ],
    },
)
