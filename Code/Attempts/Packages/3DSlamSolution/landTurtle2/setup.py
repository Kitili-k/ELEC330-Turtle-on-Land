from setuptools import setup
from glob import glob

package_name = 'landTurtle2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
    ('share/' + package_name+'/worldfile/', glob('worldfile/*')),
    ('share/' + package_name+'/textures/', glob('/textures/*')),
  	('share/' + package_name+'/meshes/collision/', glob('meshes/collision/*')),
  	('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),
    ('share/' + package_name+'/configfiles/', glob('configfiles/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
