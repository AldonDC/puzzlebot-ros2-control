from setuptools import find_packages, setup

package_name = 'detector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfonso',
    maintainer_email='A00838034@tec.mx',
    description='Paquete de visión para detección de línea, color y tráfico',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_detector = detector_pkg.line_detector:main',
            'traffic_detector = detector_pkg.traffic_detector:main',
            'angular_error_node = detector_pkg.angular_error_node:main',
            'debug_visualizer = detector_pkg.debug_visualizer:main'  # si agregas algo interactivo
        ],
    },
)
