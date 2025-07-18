from setuptools import find_packages, setup

package_name = 'img_publisher'

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
    maintainer='furkan',
    maintainer_email='furkan.artan2014@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            	'web_node = img_publisher.webcam_node:main',
		'sub_node = img_publisher.img_sub_node:main',
        ],
    },
)
