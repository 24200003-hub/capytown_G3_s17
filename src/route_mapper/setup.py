from setuptools import setup

package_name = 'route_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/mapping.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='CapyTown Team',
    maintainer_email='capytown@example.com',
    description='Memoria de recorrido y mapeo simple LiDAR/Odom para CapyTown.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_mapper_node = route_mapper.route_mapper_node:main',
        ],
    },
)
