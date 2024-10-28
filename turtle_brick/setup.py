from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengxiao Han',
    maintainer_email='hanzx@u.northwestern.edu',
    description='TODO: Package description',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arena = turtle_brick.arena:main',
            'turtle_robot = turtle_brick.turtle_robot:main',
            'debug_goal_pose = turtle_brick.debug_goal_pose:main',
            'debug_tilt = turtle_brick.debug_tilt:main',
        ],
    },
)
