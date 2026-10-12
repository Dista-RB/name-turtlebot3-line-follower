from setuptools import find_packages, setup

package_name = 'line_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dista',
    maintainer_email='disthkantc@gmail.com',
    description=('Python vision node: finds a blue line in the camera image '
                 'and publishes a steering error in [-1, 1]'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = line_vision.vision_node:main',
        ],
    },
)
