from setuptools import find_packages, setup

package_name = 'base_station_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sashang25',
    maintainer_email='sashang256@gmail.com',
    description='Base station GUI for the Mars rover relay-mesh project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_station_gui = base_station_gui.base_station_gui:main',
        ],
    },
)
