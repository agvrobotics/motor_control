from setuptools import find_packages, setup

package_name = 'lidar_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2.launch.py']),
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
        ],
    },
)
