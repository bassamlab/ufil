from setuptools import find_packages, setup

package_name = 'ufil_osn_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/classifier_projection.py']),
        ('lib/' + package_name, [package_name+'/conf_fusion.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='westhoff',
    maintainer_email='westhoff@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'classifier = ufil_osn_classifier.classifier_member_function:main',        ],
    },
)
