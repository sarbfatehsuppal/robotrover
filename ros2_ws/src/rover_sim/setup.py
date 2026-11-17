from setuptools import find_packages, setup

package_name = 'rover_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sarbfateh Suppal',
    maintainer_email='sarbfatehsuppal@users.noreply.github.com',
    description='Rocker-bogie physics estimate and simulated plant for the rover.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_node = rover_sim.sim_node:main',
            'climb_analysis = rover_sim.climb_analysis:main',
        ],
    },
)
