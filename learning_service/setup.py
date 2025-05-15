from setuptools import find_packages, setup

package_name = 'learning_service'

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
    maintainer_email='promise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'addInt_client = learning_service.addInt_client:main',
            'addInt_service = learning_service.addInt_service:main'
        ],
    },
)
