from setuptools import find_packages, setup

package_name = 'percussion_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Theo Dom',
    maintainer_email='theo.dom@student.kuleuven.be',
    description='Perception nodes for percussion system',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_service_node = percussion_perception.capture_service_node:main',
        ],
    },
)