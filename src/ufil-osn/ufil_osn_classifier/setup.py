from glob import glob

from setuptools import find_packages, setup

package_name = 'ufil_osn_classifier'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/models', glob('models/yolo11s.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Schäfer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    description='Classifies the objects of the UFIL outdoor sensor node (OSN) tracker using '
                'YOLO on camera images and fuses the camera classification into the object '
                'list.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'classifier = ufil_osn_classifier.classifier_member_function:main',
        ],
    },
)
