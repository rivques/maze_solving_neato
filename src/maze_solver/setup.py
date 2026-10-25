from setuptools import find_packages, setup

package_name = 'maze_solver'

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
    maintainer='rivques',
    maintainer_email='web@rivques.dev',
    description='CompRobo computer vision project - solving mazes with a Neato',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maze_runner = maze_solver.maze_runner:main'
        ],
    },
)
