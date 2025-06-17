from setuptools import setup
from glob import glob
import os

package_name = 'radiacode_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rove',
    maintainer_email='capra@ens.etsmtl.com',
    description='ROS 2 Python driver for RadiaCode',
    license='MIT',
    entry_points={
        'console_scripts': [
            'radiacode_node = scripts.radiacode_node:main',
        ],
    },
)
