from setuptools import find_packages, setup
from glob import glob

package_name = 'me333_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    	('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zihling',
    maintainer_email='zihling@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'talker = me333_pkg.publisher_member_function:main',
        'listener = me333_pkg.subscriber_member_function:main',
        'hello_node = me333_pkg.hello_node:main',
        'counter_publisher = me333_pkg.counter_publisher:main',
        'echo_subscriber = me333_pkg.echo_subscriber:main',
        'speed_monitor = me333_pkg.speed_monitor:main',
        'speed_sensor = me333_pkg.speed_sensor:main',
        ],
    },
)
