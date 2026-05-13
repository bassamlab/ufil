from setuptools import find_packages, setup

package_name = 'ufil_pcl_map_builder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Schaefer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    description='Creates osm maps based on points extracted form lidar survays.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ufil_pcl_map_builder_node = ufil_pcl_map_builder.ufil_pcl_map_builder_node:main'
        ],
    },
)
