from setuptools import find_packages, setup

package_name = 'turtlesim_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=['turtlesim_odom'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryana',
    maintainer_email='ryantcliff@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'turtle_odom_publisher = turtlesim_odom.turtle_odom_publisher:main',
		'odom_listener = turtlesim_odom.odom_listener:main',
        ],
    },
)
