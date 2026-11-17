from setuptools import find_packages, setup

package_name = 'rover_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sarbfateh Suppal',
    maintainer_email='sarbfatehsuppal@users.noreply.github.com',
    description='Laptop side of the rover: keyboard teleop, mission upload, odom to TF.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop = rover_station.teleop:main',
            'mission = rover_station.mission:main',
            'odom_tf = rover_station.odom_tf:main',
        ],
    },
)
