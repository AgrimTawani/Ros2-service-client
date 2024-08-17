from setuptools import find_packages, setup

package_name = 'turtlesim_spawner'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawn_service = turtlesim_spawner.turtle_spawn_service:main',
            'turtle_spawn_client = turtlesim_spawner.turtle_spawn_client:main',
        ],
    },
)
