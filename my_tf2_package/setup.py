from setuptools import setup

package_name = 'my_tf2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Example package for publishing static TF2 transforms',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf2_broadcaster = my_tf2_package.static_tf2_broadcaster:main',
            'dynamic_tf2_broadcaster = my_tf2_package.dynamic_tf2_broadcaster:main',
            'static_tf2_broadcaster_modify = my_tf2_package.static_tf2_broadcaster_modify:main',
            'static_tf2_broadcaster_add = my_tf2_package.static_tf2_broadcaster_add:main',
        ],
    },
)