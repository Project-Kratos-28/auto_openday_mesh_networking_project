import os
from glob import glob
from setuptools import setup

package_name = 'mesh_protocol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mesh_protocol']),
        ('share/mesh_protocol', ['package.xml', 'resource/photo.png']),
        (os.path.join('share', 'mesh_protocol', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Zenoh mesh network node (one per unit), test tools and launch file',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mesh_node = mesh_protocol.mesh_node:main',
            'move_unit = mesh_protocol.tools:move_unit',
            'send_cmd = mesh_protocol.tools:send_cmd',
            'mesh_monitor = mesh_protocol.tools:mesh_monitor',
            'selftest = mesh_protocol.selftest:main',
            'mission = mesh_protocol.mission:main',
            'mesh_ctl = mesh_protocol.tools:mesh_ctl'
        ],
    },
)
