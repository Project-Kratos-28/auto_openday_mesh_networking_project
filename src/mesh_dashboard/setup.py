from setuptools import setup

package_name = 'mesh_dashboard'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mesh_dashboard']),
        ('share/mesh_dashboard', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    description='Mission-control web dashboard for the relay mesh',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard = mesh_dashboard.server:main',
        ],
    },
)
