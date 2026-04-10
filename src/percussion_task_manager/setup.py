from setuptools import setup

package_name = 'percussion_task_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Theo Dom',
    maintainer_email='theo.dom@student.kuleuven.be',
    description='Task manager and capture service stub for the scaffolding fixation thesis project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = percussion_task_manager.task_manager_node:main',
        ],
    },
)
