from setuptools import setup
from glob import glob

package_name = 'turtle_on_land_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch/', glob('launch/*.py')),
        ('share/' + package_name+'/urdf/', glob('urdf/*')),
        ('share/' + package_name+'/rviz/', glob('rviz/*')),
        ('share/' + package_name+'/meshes/collision/', glob('meshes/collision/*')),
        ('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),
        ('share/' + package_name+'/config/', glob('config/*')),
        ('share/' + package_name+'/gazebo/', glob('gazebo/*')),
        ('share/' + package_name+'/world/', glob('world/*')),
        ('share/' + package_name+'/src/', glob('src/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'publisher = turtle_on_land_demo.publisher:main',
                            'hslam = turtle_on_land_demo.hslam:main',
                            'rgb2 = turtle_on_land_demo.rgb2:main',
                            'rgb3 = turtle_on_land_demo.rgb3:main',
                            'pose_to_odom = turtle_on_land_demo.pose_to_odom:main',

        ],
    },
)
