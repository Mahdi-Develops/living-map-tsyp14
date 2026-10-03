from setuptools import setup

package_name = 'living_map_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Writer/Executor/Outside-Network/Command-Post logic for the Living Map challenge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'writer_node = living_map_system.writer_node:main',
            'executor_node = living_map_system.executor_node:main',
            'outside_network_node = living_map_system.outside_network_node:main',
            'command_post_node = living_map_system.command_post_node:main',
        ],
    },
)
