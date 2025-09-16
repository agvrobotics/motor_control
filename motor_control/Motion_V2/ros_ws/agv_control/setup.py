from setuptools import find_packages, setup

package_name = 'agv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agv_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sierra-95',
    maintainer_email='michaelmachohi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = agv_control.serial_node:main',
            'keyboard_teleop = agv_control.teleop_keyboard:main',
        ],
    },
)
