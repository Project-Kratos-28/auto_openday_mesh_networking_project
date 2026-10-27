import os
from glob import glob
from setuptools import setup

package_name = 'mesh_link_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mesh_link_model']),
        ('share/mesh_link_model', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Line of sight + range link model, radio channel and stand-in failure injector',
    license='MIT',
    entry_points={
        'console_scripts': [
            'link_model = mesh_link_model.link_model:main',
            'radio_channel = mesh_link_model.radio_channel:main',
            'passthrough_injector = mesh_link_model.passthrough_injector:main'
        ],
    },
)
