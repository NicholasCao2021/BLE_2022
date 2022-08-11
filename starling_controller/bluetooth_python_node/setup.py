from setuptools import setup

package_name = 'bluetooth_python_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NicholasCao',
    maintainer_email='ak21472@bristol.ac.uk',
    description='Python ROS2 node template for running offboard',
    license='MIT License @ NicholasCao 2022',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = bluetooth_python_node.main:main'
        ],
    },
)
