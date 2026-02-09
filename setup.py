from setuptools import setup, find_packages

package_name = 'fukuro_sim2d'

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
    maintainer='shluf',
    maintainer_email='luthfisalis09@gmail.com',
    description='Simulasi 2D robot omniwheel dengan Pygame dan ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = fukuro_sim2d.simulation_node:main',
        ],
    },
)
