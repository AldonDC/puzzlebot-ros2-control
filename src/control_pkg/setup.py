from setuptools import find_packages, setup

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfonso',
    maintainer_email='A00838034@tec.mx',
    description='Paquete de control para el Puzzlebot (PID, seguimiento de línea, semáforo)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_controller = control_pkg.line_follower_controller:main',
            'odometry_node = control_pkg.odometry_node:main',
            'path_generator_node = control_pkg.path_generator_node:main',
            'pid_controller_node = control_pkg.pid_controller_node:main',
            'traffic_light_controller = control_pkg.traffic_light_controller:main',
            'path_generator_traffic = control_pkg.path_generator_traffic:main',
        ],
    },
)