import os
from glob import glob
from setuptools import setup

package_name = 'mesh_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mesh_common']),
        ('share/mesh_common', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Shared names, Zenoh keys and packet format (CRC32) for the Mars mesh',
    license='MIT',
    entry_points={
        'console_scripts': [
            
        ],
    },
)
