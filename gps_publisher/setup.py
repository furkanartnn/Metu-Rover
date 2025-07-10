from setuptools import setup

package_name = 'gps_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
	'setuptools',
	'pyserial',
	'websockets',
    ],
    zip_safe=True,
    maintainer='furkan',
    maintainer_email='furkan@example.com',
    description='GPS verisini ROS 2 topic olarak yayınlayan node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_publisher.gps_node:main',
	    'gps_bridge = gps_publisher.gps_bridge:main',
        ],
    },
)

