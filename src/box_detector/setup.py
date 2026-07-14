from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'box_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Esto asegura que ROS 2 encuentre tus carpetas 'launch' y 'config' al compilar
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nino Fermin Briceño Huaman',
    maintainer_email='nino.briceno@esan.edu.pe',
    description='Paquete de percepcion y conteo de cajas usando LiDAR para el Reto',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Comando de terminal : Ruta del script y funcion a ejecutar
            'box_detector_node = box_detector.box_detector:main',
            'box_detector = box_detector.box_detector:main',
            'metrics_logger = box_detector.metrics_logger:main',
        ],
    },
)