from setuptools import setup

package_name = 'ssl_wheel_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Schäfer',
    maintainer_email='schaefer@embedded.rwth-aachen.de',
    author='Hendrik Steidl',
    author_email='hendrik.steidl@rwth-aachen.de',
    description='Wheel tracking based on wheel detections.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ssl_wheel_tracking_node = ssl_wheel_tracking.wheel_tracking_node:main'
        ],
    },
)
