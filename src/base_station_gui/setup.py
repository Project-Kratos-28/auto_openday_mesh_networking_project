from setuptools import setup

package_name = 'base_station_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/base_station_gui']),
        ('share/base_station_gui', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Base station GUI (PyQt6) - placeholder with the mesh ON/OFF toggle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui = base_station_gui.app:main',
        ],
    },
)
