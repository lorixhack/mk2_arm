from glob import glob
from setuptools import setup

package_name = 'simplified_arm_assembly'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf')),
        (f'share/{package_name}/meshes', glob('meshes/*.STL')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Example ROS 2 Python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = simplified_arm_assembly.my_node:main',
        ],
    },
)

