from setuptools import find_packages, setup

package_name = 'learning_fsmbt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='promise',
    maintainer_email='2441447381@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'learning_fsm = learning_fsmbt.learning_fsm:main',
            'learning_bt = learning_fsmbt.learning_bt:main',
            'learning_grasp = learning_fsmbt.learning_grasp:main',
        ],
    },
)
