from setuptools import find_packages, setup
from glob import glob

package_name = 'bird_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jake_161',
    maintainer_email='62108812+jake161@users.noreply.github.com',
    description='URDF, Launch, and RVIZ files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
