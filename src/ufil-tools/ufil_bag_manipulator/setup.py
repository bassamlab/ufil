from setuptools import find_packages, setup

package_name = 'ufil_bag_manipulator'

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
    maintainer='Simon Schäfer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    description='Shifts ROS2 bag timestamps and merges two bags',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trim_shift_rosbags = ufil_bag_manipulator.bagShifter:main',
             'merge_rosbags = ufil_bag_manipulator.bagMerger:main',
        ],
    },
)
