from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'world'), glob('world/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharva',
    maintainer_email='atharva.abhi.kulkarni@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner = dwa_planner.planner_node:main',
            'goal = dwa_planner.goal_publisher:main',
            'transform = dwa_planner.transform_odom:main',
            'converter = dwa_planner.twist_joint:main',
            'wheel_odometry = dwa_planner.wheel_odometry:main',
        ],
    },
)
