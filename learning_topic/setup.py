from setuptools import find_packages, setup

package_name = 'learning_topic'

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
            'topic_publisher = learning_topic.topic_publisher:main',
            'topic_subscriber = learning_topic.topic_subscriber:main',
        ],
    },
)
