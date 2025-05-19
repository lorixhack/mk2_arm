from glob import glob

from setuptools import setup

package_name = 'can_sender_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team ISAAC',
    maintainer_email='team.isaac@polito.it',
    description='ROS2 code for ReseQ robot',
    license='GNU GPL v3.0',
    entry_points={
        'console_scripts': [
            'can_sender = can_sender_pkg.can_sender:main',
        ],
    },
)
