from setuptools import setup

package_name = 'pointcloud_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'launch/table_demo.launch.xml', 'config/pcl.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Demonstrate Point Cloud Processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'table = pointcloud_demo.tablefinder:table_entry'
        ],
    },
)
