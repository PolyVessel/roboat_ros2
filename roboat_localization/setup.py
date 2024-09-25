from setuptools import setup

package_name = 'roboat_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboat',
    maintainer_email='<>',
    description='roboat thinking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = roboat_localization.localization:main'
        ],
    },
)
