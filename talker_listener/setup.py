from setuptools import setup

package_name = 'talker_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monsieurtusk',
    maintainer_email='monsieurtusk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talkerNode_m1 = talker_listener.talker_node_m1:main',
            'talkerNode_m2 = talker_listener.talker_node_m2:main',
            'talkerNode_m1m2 = talker_listener.talker_node_m1m2:main',
            'listenerNode = talker_listener.listener_node:main',
            'rapportNode = talker_listener.rapport_node:main',
            'visualisationNode = talker_listener.visualisation_data_node:main'
        ],
    },
)
