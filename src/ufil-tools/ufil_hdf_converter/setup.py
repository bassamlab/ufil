from setuptools import find_packages, setup

package_name = 'ufil_hdf_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'progressbar2',
    ],
    zip_safe=True,
    maintainer='Simon Schäfer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    description='Contains the code used to convert a rosbag2 to a Ufil supported HDF5.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ufil_hdf_converter-node = ufil_hdf_converter.ufil_hdf_converter_node:main'
        ],
    },
)
