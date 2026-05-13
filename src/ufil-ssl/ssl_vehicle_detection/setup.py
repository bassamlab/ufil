from glob import glob
import os

from setuptools import setup

package_name = 'ssl_vehicle_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Schäfer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    author='Hendrik Steidl',
    author_email='hendrik.steidl@rwth-aachen.de',
    description='Vehicle detection based on wheel tracks.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ssl_vehicle_detection_node = ssl_vehicle_detection.vehicle_detection_node:main'
        ],
    },
)
