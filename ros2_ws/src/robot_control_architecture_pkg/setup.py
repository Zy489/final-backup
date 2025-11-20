from setuptools import setup

package_name = 'robot_control_architecture_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/robot_control_architecture.launch.py']),
        ('share/' + package_name + '/config',
         ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zifan Yang',
    maintainer_email='zy489@cornell.edu',
    description='Hardcoded quiz robot (B-C-B) with waiting times and TTS',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_quiz_demo_hardcode = robot_control_architecture_pkg.robot_quiz_demo_hardcode:main',
            'simple_tts_node = robot_control_architecture_pkg.simple_tts_node:main',
        ],
    },
)
