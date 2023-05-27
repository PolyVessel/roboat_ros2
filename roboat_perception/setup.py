from setuptools import setup

package_name = 'roboat_perception'
submodules = package_name + "/lib"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboat',
    maintainer_email='<>',
    description='Roboats Sensors and Perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps = roboat_perception.gps:main',
            'radio = roboat_perception.radio:main'
        ],
    },
)
