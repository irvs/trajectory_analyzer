from setuptools import setup
import os
from glob import glob

package_name = 'traj_follow_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.png')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Plot controller tracking (reference/feedback/error) from JointTrajectoryControllerState.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot = traj_follow_plotter.traj_follow_plotter_node:main',
            'video_player = traj_follow_plotter.video_player_node:main',
            'generate_rviz_config = traj_follow_plotter.generate_rviz_config:main',
        ],
    },
)
