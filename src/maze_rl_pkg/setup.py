from setuptools import find_packages, setup

package_name = 'maze_rl_pkg'

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
    maintainer='marcel',
    maintainer_email='saragihmarcel34@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maze_rl_node = maze_rl_pkg.maze_rl_node:main',
            'train = maze_rl_pkg.train:main',
            'eval = maze_rl_pkg.eval:main'
        ],
    },
)
