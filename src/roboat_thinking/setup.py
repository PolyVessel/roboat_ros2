from setuptools import setup
from setuptools.command.develop import develop
from setuptools.command.install import install
from subprocess import check_call
import os
import sys


def compile_protos():
    protos_dir = "../../protos"
    generated_files_dir = "./roboat_thinking/pb"
    
    check_call(f"rm -f {generated_files_dir}/*_pb2.py", shell=True)
    
    files_in_protos_dir = os.listdir(protos_dir)
    
    proto_files = [file for file in files_in_protos_dir if file.endswith('.proto')]
    
    if len(proto_files) == 0:
        print("No Protobuf Files found to compile")
        return
    
    check_call(f"protoc --python_out={generated_files_dir} \
                       --proto_path {protos_dir} \
                       --experimental_allow_proto3_optional \
                       {protos_dir}/*.proto", shell=True)

class PostDevelopCommand(develop):
    """Post-installation for development mode."""
    def run(self):
        compile_protos()
        develop.run(self)

class PostInstallCommand(install):
    """Post-installation for installation mode."""
    def run(self):
        compile_protos()
        install.run(self)

package_name = 'roboat_thinking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f"{package_name}/pb"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboat',
    maintainer_email='<>',
    description='Internal processes of the boat',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder = roboat_thinking.recorder:main',
            'msg_decoder = roboat_thinking.msg_decoder:main',
        ],
    },
    cmdclass={
        'develop': PostDevelopCommand,
        'install': PostInstallCommand,
    },
)
